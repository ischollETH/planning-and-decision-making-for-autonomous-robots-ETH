# Implementation inspired by notebook from the underactuated robotics course at MIT:
# https://colab.research.google.com/github/RussTedrake/underactuated/blob/master/exercises/planning/rrt_planning/rrt_planning.ipynb

import numpy as np
from dg_commons.sim.models.spacecraft import SpacecraftState, SpacecraftCommands
from dg_commons.sim.models.model_utils import apply_acceleration_limits, apply_rot_speed_constraint
from scipy.integrate import solve_ivp
from shapely.geometry import Point
from shapely.affinity import affine_transform, rotate, translate
from pdm4ar.exercises.final21.halton_sequence import HaltonSequence
from pdm4ar.exercises.final21.neighbor import Tree

from dg_commons.sim.models.obstacles_dyn import DynObstacleModel, DynObstacleState, DynObstacleCommands

from pdm4ar.exercises.final21.debug_tools import Plotter
from pdm4ar.exercises.final21.steering_v2 import SteeringOptimizer

DEBUG = False
MAX_TTL = 8

class RRT:
    class Node:
        def __init__(self, st, time=0):
            self.st = st  # We assume st is a SpacecraftState
            self.time = time
            self.ttl = MAX_TTL  # Defines the maximum number of consecutive unsucessful branching attempts before pruning
            self.nr_children = 0
            self.parent = None
            self.actions = []
            self.path = []

    def __init__(self, start, goal, static_obstacles, dynamic_obstacles, dt, sc_parameters, sc_geometry):
        self.start = self.Node(start, time=0)
        self.goal = goal.goal
        self.dim = 6  # x, y, psi, v_x, v_y, dpsi
        self.sc_parameters = sc_parameters
        self.sc_geometry = sc_geometry
        self.static_obstacles = static_obstacles
        self.bounds = self.set_bounds()
        self.dynamic_obstacles_at_origin = self.get_dyn_obstacles_at_origin(dynamic_obstacles)
        self.dynamic_obstacles_state_traj = [[dyn_ob.state for dyn_ob in dynamic_obstacles]]
        self.dynamic_obstacles_polygon_traj = [[dyn_ob.occupancy for dyn_ob in dynamic_obstacles]]
        self.dt = dt
        self.kd_tree = None  # TODO, may be useful for find_nearest_neighbor
        self.path_tree = []  # TODO, possibly find a more efficient way to represent the path tree
        # Sampling
        self.rng = np.random.default_rng()
        self.sampling_seq = HaltonSequence(min_bounds=self.bounds[0], max_bounds=self.bounds[1], dimensions=self.dim, size=10000)
        self.halton_to_random_ratio = 10
        self.nr_samples = 0

        self.steering_optim = SteeringOptimizer(N=22, sg=self.sc_geometry, sp=self.sc_parameters, static_obstacles=static_obstacles)
        if DEBUG:
            self.plotter = Plotter(self)

    def plan(self):
        """Plans a path from the start position to the goal zone"""
        self.path_tree = [self.start]
        goal_reached = False
        while not goal_reached:
            x_rand = self.sample_state()
            x_nearest, x_nearest_ind, dist = self.get_nearest_neighbor(x_rand)
            if dist < 1:
                continue

            # get the actions dictated by the steering method
            # actions = self.steer(x_nearest.st, x_rand, 30)
            # get the actions dictated by the steering method
            actions, x_path, success = self.steer_opti(x_nearest.st, x_rand)
            if not success:  # if not feasible
                self.check_tree_pruning(x_nearest_ind, 1)
                continue  # resample

            # simulate the actions to get the path of visited states
            # with the current implementation, this could be taken directly from the steer function, but is left like
            # this for compatibility with other steering approaches.
            # state_path = self.simulate(x_nearest.st, actions)
            state_path = x_path

            # take the last state in the state path to be used for the new node.
            x_new = state_path[-1]

            # add the new node if state path does not collide.
            target_time = x_nearest.time + len(actions)*self.dt
            if not (self.check_collision_path(state_path) or self.check_collision_path_dyn(state_path, target_time)):
                # if the new node is in the goal, we will terminate
                if self.goal_on_path(state_path):
                    goal_reached = True
                # create node from new state and add to the tree.
                x_new = self.Node(x_new)
                x_new.actions = actions
                x_new.path = state_path
                x_new.parent = self.path_tree[x_nearest_ind]
                x_new.time = target_time
                self.path_tree.append(x_new)
                self.path_tree[x_nearest_ind].ttl = MAX_TTL
                self.path_tree[x_nearest_ind].nr_children += 1
                if DEBUG:
                    self.plotter.update(state_path)
            else:  # draw invalid paths gray
                self.check_tree_pruning(x_nearest_ind, 2)
                if DEBUG:
                    self.plotter.update(state_path, True)
        if DEBUG:
            self.plotter.pause()
        return self.final_path(x_new)  # returns the branch leading to the final node

    def check_tree_pruning(self, node_idx, lifes_lost):
        self.path_tree[node_idx].ttl -= lifes_lost
        is_not_root = (node_idx != 0)
        has_no_children = (self.path_tree[node_idx].nr_children == 0)
        is_useless = (self.path_tree[node_idx].ttl <= 0)
        prune_leaf = is_not_root and has_no_children and is_useless
        if prune_leaf:
            parent = self.path_tree[node_idx].parent
            parent.nr_children -= 1
            self.path_tree.pop(node_idx)
        return prune_leaf

    def steer_opti(self, from_state, to_state):
        from_state = from_state.as_ndarray()
        to_state = to_state.as_ndarray()
        U, X, success = self.steering_optim.steer(from_state, to_state)

        # truncation method if use stage cost
        max_dev = 5
        for i in range((X.shape[1]) - 1):
            if i < 2:
                continue  # dont want empty trajectories
            if np.max(np.abs(X[:, i] - np.abs(X[:, -1]))) <= max_dev:
                X = X[:, :i]
                U = U[:, :(i-1)]
                # print('Truncated')
                break

        actions = []
        path = []
        for i in range((X.shape[1]) - 1):
            actions.append(SpacecraftCommands(acc_left=U[1, i], acc_right=U[0, i]))
            path.append(SpacecraftState.from_array(X[:, i+1]))
        # print(len(actions))

        return actions, path, success

    def steer(self, from_state, to_state, max_time_step):
        """
        Some steering algorithm to bring the system from from_state to to_state. Returns the actions
        Steers only towards the position, ignoring orientation and velocity
        Implements a somewhat hierarchical control scheme using two virtual inputs, u_rot and u_thrust. u_rot has the
        highest priority and steers the spaceship heading to be towards the target (adjusted for current velocity)
        u_thrust propels the spaceship according to how well aligned the ship is with the target heading.

        NOTE: This method is very long and likely to be replaced by a smarter steering scheme, so do not worry too much
        about how it works. I also holds several functions that would allow for slightly different approaches and behaviour.
        """

        # for some reason when moving in negative x direction, the approach breaks down.
        # investigate this next.
        st = from_state
        rot_p = 8
        rot_d = 0.1
        rot_i = 0.005
        angle_error_int = 0

        thrust_p = 3
        thrust_d = 8

        st_dif = to_state - st
        step = 0
        actions = []

        def signed_vec_angle(vec1, vec2):
            """
            Calculates a signed angle between two vectors
            """
            return np.arctan2(vec1[0] * vec2[1] - vec1[1] * vec2[0],
                              vec1[0] * vec2[0] + vec1[1] * vec2[1])

        def clip_u_t(u_t, u_rot):
            """
            Clips the
            """
            if u_t < -20 + abs(u_rot):
                return min(0, -20 + abs(u_rot))
            if u_t > 20 - abs(u_rot):
                return max(0, 20 - abs(u_rot))
            return u_t

        def clip_angle(angle):
            if angle > np.pi / 2:
                return np.pi / 2 - angle
            if angle < -np.pi / 2:
                return -np.pi / 2 - angle
            return angle

        def get_angle_error(state, state_dif, method='vel'):
            sgn = 1
            abs_ang_err = signed_vec_angle(np.array([np.cos(state.psi), np.sin(state.psi)]),
                                           np.array([state_dif.x, state_dif.y]))
            if method == 'abs_ang':
                ang_err = abs_ang_err
            elif method == 'vel':
                rot_mat = np.array(
                    [[np.cos(state.psi), -np.sin(state.psi)], [np.sin(state.psi), np.cos(state.psi)]])
                if abs(state.vx + state.vy) < 0.001:
                    ang_err = signed_vec_angle(np.array([np.cos(state.psi), np.sin(state.psi)]),
                                               np.array([state_dif.x, state_dif.y]))
                else:
                    vel_world = np.matmul(rot_mat, np.array([state.vx, state.vy]))
                    ang_err = signed_vec_angle(vel_world, np.array([state_dif.x, state_dif.y]))
                # idea, get an error of pi to be equal to 0
                # an error of >pi/2 but <pi should map to angle-pi
                # an error of <-pi/2 but >-pi should map to angle +pi
                # error <pi/2 and >-pi/2 can map directly
                if np.pi / 2 < ang_err <= np.pi:
                    ang_err = ang_err - np.pi
                elif -np.pi / 2 > ang_err >= -np.pi:
                    ang_err = ang_err + np.pi
                st_pos_dif_bf = np.matmul(rot_mat.transpose(), np.array([st_dif.x, st_dif.y]))
                sgn = st_pos_dif_bf[0]
            return ang_err, sgn

        def get_angle_error_alt(state, state_dif):
            mu = 0.5
            rot_mat = np.array(
                [[np.cos(state.psi), -np.sin(state.psi)], [np.sin(state.psi), np.cos(state.psi)]]).transpose()
            target_dif_bf = np.matmul(rot_mat, np.array([state_dif.x, state_dif.y]))
            vel_vector_bf = np.array([state.vx, state.vy])
            target_heading = target_dif_bf - mu * vel_vector_bf
            ang_err = np.arctan2(target_heading[1], target_heading[0])
            if np.pi / 2 < ang_err <= np.pi:
                ang_err = ang_err - np.pi
            elif -np.pi / 2 > ang_err >= -np.pi:
                ang_err = ang_err + np.pi
            thrst = target_heading[0]
            return ang_err, thrst

        while max(abs(st_dif.y), abs(st_dif.x)) > 0.5 and step < max_time_step:
            # the approach is to control based on the difference in the angle between the velocity vector and the vector
            # to the target point

            angle_to_target = np.arctan2(st_dif.y, st_dif.x)
            dist_to_target = np.sqrt((st_dif.x ** 2 + st_dif.y ** 2))

            # angle_error, sign = get_angle_error(st, st_dif, method='vel')
            angle_error, thrust = get_angle_error_alt(st, st_dif)

            angle_error_int += angle_error

            if step == 0:
                angle_error_dif = 0
            else:
                angle_error_dif = (angle_error - angle_error_old) / self.dt

            angle_error_old = angle_error

            u_rot = angle_error * rot_p + angle_error_dif * rot_d + rot_i * angle_error_int - st.dpsi
            # st_pos_dif_bf = np.matmul(rot_mat.transpose(), np.array([st_dif.x, st_dif.y]))
            # u_thrust = thrust_p * (sign * np.cos(angle_error) ** 3) - thrust_d * st.vx

            u_thrust = thrust_p * thrust - thrust_d * st.vx

            if step == 10:
                temp = 1
            # Try: cap the thrust to prioritise rotation
            ar = (u_rot + clip_u_t(u_thrust, u_rot)) / 2
            al = (clip_u_t(u_thrust, u_rot) - u_rot) / 2
            actions.append(SpacecraftCommands(acc_left=al, acc_right=ar))
            st = self.simulate(st, [actions[step]])[0]
            st_dif = to_state - st
            step += 1
        return actions

    def set_bounds(self):
        """Sets the bounds of the RRT object to a minimum and a maximum array of the given dimensionality. The order is
        x, y, psi, v_x, v_y, dpsi"""
        boundary_obstacle = self.static_obstacles[0].shape.bounds  # Assuming the boundary is always the first obstacle
        x_min = boundary_obstacle[0]
        y_min = boundary_obstacle[1]
        x_max = boundary_obstacle[2]
        y_max = boundary_obstacle[3]
        psi_min = 0
        psi_max = 2 * np.pi
        v_max = self.sc_parameters.vx_limits[1]
        v_min = self.sc_parameters.vx_limits[0]
        dpsi_max = self.sc_parameters.dpsi_limits[1]
        dpsi_min = self.sc_parameters.dpsi_limits[0]

        min_bounds = np.array([x_min, y_min, psi_min, v_min, v_min, dpsi_min])
        max_bounds = np.array([x_max, y_max, psi_max, v_max, v_max, dpsi_max])

        return [min_bounds, max_bounds]

    def sample_state(self):
        """Samples a random state in the free space"""
        valid_state_found = False
        state = SpacecraftState(x=self.goal.centroid.x, y=self.goal.centroid.y, psi=0, vx=0, vy=0, dpsi=0)

        while not valid_state_found:
            if self.nr_samples % self.halton_to_random_ratio == 0:
                # print("Sample randomly around goal")
                xy_arr = self.rng.uniform(low=self.goal.bounds[:1], high=self.goal.bounds[2:])
                state_arr = self.rng.uniform(low=self.bounds[0], high=self.bounds[1])
                state = SpacecraftState(x=xy_arr[0], y=xy_arr[1], psi=state_arr[2], vx=state_arr[3],
                                        vy=state_arr[4],
                                        dpsi=state_arr[5])

            else:
                # print("Sample from Halton Sequence")
                state_arr = self.sampling_seq.get_next_sample()
                state = SpacecraftState(x=state_arr[0], y=state_arr[1], psi=state_arr[2], vx=state_arr[3], vy=state_arr[4],
                                dpsi=state_arr[5])

            valid_state_found = not self.check_collision_state(state)[0]
            self.nr_samples += 1

        return state

    def get_nearest_neighbor(self, x_rand):
        """
        Find the nearest state already in the graph. More advanced implementation uses a KD-Tree,
        then only allows solutions that lie within a favorable region
        """

        # def get_node_distance(x1, x2):
        #   dif = x1 - x2
        #    dist = (dif.x ** 2 + dif.y ** 2)
        #    return dist

        # dist_list = [get_node_distance(x_rand, x_tree.st) for x_tree in self.path_tree]
        # nn_index = np.argmin(dist_list)
        # return self.path_tree[nn_index], nn_index, np.sqrt(dist_list[
        #                                                      nn_index])  # index in self.path_tree of the nearest neighbor. Can change if we find a better path_tree
        nodes = np.zeros((len(self.path_tree), 6))
        for i, x_tree in enumerate(self.path_tree):
            nodes[i] = x_tree.st.as_ndarray()
        tree = Tree(nodes)
        k = len(self.path_tree)
        if k > 100:
            k = 100
        best_idx, best_dist = tree.find_reachable_neighbor(x_rand.as_ndarray(), k)
        return self.path_tree[best_idx], best_idx, best_dist

    # --------------------------------- Spaceship simulation and dynamics --------------------------------------------
    # ----------------------------------------------------------------------------------------------------------------
    def simulate(self, start: SpacecraftState, actions, time_div=1):
        """Generates a path of states based on the starting state and the actions.
        Actions are typically generated by a steering function. If a finer or coarser time discretization than self.dt
        is desired, the time_div parameter can be used."""
        n_states = start.get_n_states()

        def _dynamics(t, y):
            """
            Wrapper for dynamics to solve using solve_ivp
            y contains states and actions in an array: [states, actions]
            """
            st = SpacecraftState.from_array(y[:n_states])
            act = SpacecraftCommands(
                acc_left=y[SpacecraftCommands.idx["acc_left"] + n_states],
                acc_right=y[SpacecraftCommands.idx["acc_right"] + n_states],
            )  # SpacecraftCommands.from_array(y[n_states:])

            dx = self.dynamics(x0=st, u=act)
            du = np.zeros([act.get_n_commands()])
            return np.concatenate([dx.as_ndarray(), du])

        path = []
        state = start
        for ac in actions:  # here we can probably do some repmat stuff if we need finer control. Alternatively add
            # another for loop.
            for step in range(time_div):
                y0 = np.concatenate([state.as_ndarray(), ac.as_ndarray()])
                result = solve_ivp(fun=_dynamics, t_span=(0.0, float(self.dt / time_div)), y0=y0)
                state = SpacecraftState.from_array(result.y[:n_states, -1])
                path.append(state)
        return path

    def dynamics(self, x0: SpacecraftState, u: SpacecraftCommands) -> SpacecraftState:
        """Dynamics of the spacecraft. Copied over from simulator"""
        acc_lx = apply_acceleration_limits(u.acc_left, self.sc_parameters)
        acc_rx = apply_acceleration_limits(u.acc_right, self.sc_parameters)
        acc_sum = acc_lx + acc_rx
        acc_diff = acc_rx - acc_lx

        vx = x0.vx
        vy = x0.vy
        costh = np.cos(x0.psi)
        sinth = np.sin(x0.psi)
        dx = vx * costh - vy * sinth
        dy = vx * sinth + vy * costh

        ax = acc_sum + x0.vy * x0.dpsi
        ay = -x0.vx * x0.dpsi
        ddpsi = self.sc_geometry.w_half * self.sc_geometry.m / self.sc_geometry.Iz * acc_diff
        ddpsi = apply_rot_speed_constraint(x0.dpsi, ddpsi, self.sc_parameters)
        return SpacecraftState(x=dx, y=dy, psi=x0.dpsi, vx=ax, vy=ay, dpsi=ddpsi)

    # ------------------------------------------ Dynamic Obstacles ----------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------
    def get_dyn_obstacles_at_origin(self, observation_list):
        polygons_at_origin = []
        for obs in observation_list:
            state = obs.state
            polygon = obs.occupancy
            # reset rotation
            polygon = rotate(polygon, -state.psi, origin=(state.x, state.y), use_radians=True)
            polygon = translate(polygon, xoff=-state.x, yoff=-state.y)
            polygons_at_origin.append(polygon)

        # matplotlib.use('TkAgg')
        # plt.close('all')
        # plt.figure()
        # ax = plt.gca()
        # shapely_viz = ShapelyViz(ax)
        #
        # for poly in polygons_at_origin:
        #     shapely_viz.add_shape(poly, zorder=ZOrders.ENV_OBSTACLE)
        # plt.xlim([-5,5])
        # plt.ylim([-5,5])
        # plt.show()
        return polygons_at_origin

    def obstacle_dynamics_continuous(self, x0):
        vx, vy = x0.vx, x0.vy
        costh = np.cos(x0.psi)
        sinth = np.sin(x0.psi)
        xdot = vx * costh - vy * sinth
        ydot = vx * sinth + vy * costh

        # we assume no input. Best we can do anyway as we have no way of knowing the input.
        return DynObstacleState(x=xdot, y=ydot, psi=x0.dpsi, vx=0, vy=0, dpsi=0)

    def obstacle_dynamics_rk(self, x, Ts):
        # classical RK-4  (TODO RK-45)
        k1 = self.obstacle_dynamics_continuous(x)
        k2 = self.obstacle_dynamics_continuous(x + Ts / 2 * k1)
        k3 = self.obstacle_dynamics_continuous(x + Ts / 2 * k2)
        k4 = self.obstacle_dynamics_continuous(x + Ts * k3)

        return x + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def dyn_obs_polys_from_states(self, states):
        polygons = []
        for idx, st in enumerate(states):
            poly = self.dynamic_obstacles_at_origin[idx]
            poly_trans = rotate(poly, st.psi, origin=(0, 0), use_radians=True)
            poly_trans = translate(poly_trans, xoff=st.x, yoff=st.y)
            polygons.append(poly_trans)
        return polygons

    def reset_dyn_obs_to_time(self, time, dyn_obs):
        """
        This method should be used at the time when too large deviation between planned and observed dynamic obstacles
        is found. Specific logic should be determined for that situation, but i suggest following a call to this function
        by a collision check of the whole planned spacecraft trajectory, and determining whether to fully replan based
        on the outcome of that.
        """
        time_idx = round(time/self.dt)
        # i need to insert the current (new) observation at time, and remove the rest of the array for
        # dynamic_obstacles_state_traj and dynamic_obstacles_polygon_traj
        self.dynamic_obstacles_state_traj = self.dynamic_obstacles_state_traj[:time_idx]
        self.dynamic_obstacles_state_traj.append([dyn_ob.state for dyn_ob in dyn_obs])

        self.dynamic_obstacles_polygon_traj = self.dynamic_obstacles_polygon_traj[:time_idx]
        self.dynamic_obstacles_polygon_traj.append([dyn_ob.occupancy for dyn_ob in dyn_obs])


    def check_collision_path_dyn(self, sc_path, target_time):
        """
        Simulates dynamic obstacles up to target time (if needed. A lot of the time we will have simulated these already)
        Then performs collision checks against the dynamic obstacles at the correct time step.
        """
        if self.dynamic_obstacles_at_origin:
            while (len(self.dynamic_obstacles_state_traj)-1)*self.dt < target_time:
                # simulate more obstacle time steps
                new_obs_states = [self.obstacle_dynamics_rk(x, self.dt) for x in self.dynamic_obstacles_state_traj[-1]]
                self.dynamic_obstacles_state_traj.append(new_obs_states)
                new_obs_polys = self.dyn_obs_polys_from_states(new_obs_states)
                self.dynamic_obstacles_polygon_traj.append(new_obs_polys)

            # TODO: verify that the polygons being checked are correct.
            target_time_idx = round(target_time/self.dt)
            dyn_obs_path = self.dynamic_obstacles_polygon_traj[target_time_idx+1-len(sc_path):target_time_idx+1]
            min_dist = np.inf
            for sc_st, obs_poly in zip(sc_path, dyn_obs_path):
                collision, dist = self.check_collision_state_obs(sc_st, obs_poly)
                if collision:
                    return True, -1
                if dist < min_dist:
                    min_dist = dist
        return False
        # relation between target_time and indices:

    def check_collision_state_obs(self, state, obs, thresh=10e-2):
        """
        Returns true if collision, we might need a new function to handle dynamic obstacles
        dyn_obs is a list of polygons. Previous logic must ensure that the polygon matches the correct time
        """
        psi = state.psi
        x = state.x
        y = state.y
        transform = np.array([np.cos(psi), -np.sin(psi), np.sin(psi), np.cos(psi), x, y])
        spacecraft_geometry = self.sc_geometry.outline_as_polygon
        spacecraft_geometry = affine_transform(spacecraft_geometry, transform)
        min_dist = np.inf
        for ob in obs:
            if ob.intersects(spacecraft_geometry):
                return True, -1
            dist = ob.distance(spacecraft_geometry)
            if dist < min_dist:
                min_dist = dist
        #  now has a bit of a margin which can be adjusted using thresh
        return (min_dist < thresh), min_dist

    # ---------------------------- Colision and goal checking ----------------------------------
    # ------------------------------------------------------------------------------------------
    def check_collision_path(self, path):
        """
        Assumes a path of states and checks collision with obstacles.
        We could also think of a way to create a collision checker object, which can maintain an area of known safe
        areas. This might not actually be worth it in this case as we know exactly where the obstacles are a priori.
        """
        # This implementation is very crude. Currently we are not taking advantage of the distance
        min_dist = np.inf
        for idx, st in enumerate(path):
            collision, dist = self.check_collision_state_obs(st, [ob.shape for ob in self.static_obstacles])
            if collision:
                return True, -1
            if self.in_goal_area(st):
                if idx == len(path) - 1:
                    return False
                collision, dist = self.check_collision_state(path[idx + 1])  # make sure next state not colliding
                if not collision:
                    return False  # ignore collision of rest of path since sim. will stop anyway
            if dist < min_dist:
                min_dist = dist
        return False

    def check_collision_state(self, state):
        """
        NOTE: Currently replaced by check_collision_state_obs()
        Returns true if collision, we might need a new function to handle dynamic obstacles
        dyn_obs is a list of polygons. Previous logic must ensure that the polygon matches the correct time
        """
        psi = state.psi
        x = state.x
        y = state.y
        transform = np.array([np.cos(psi), -np.sin(psi), np.sin(psi), np.cos(psi), x, y])
        spacecraft_geometry = self.sc_geometry.outline_as_polygon
        spacecraft_geometry = affine_transform(spacecraft_geometry, transform)
        min_dist = np.inf
        for ob in self.static_obstacles:
            ob_shape = ob.shape
            if ob_shape.intersects(spacecraft_geometry):
                return True, -1
            dist = ob.shape.distance(spacecraft_geometry)
            if dist < min_dist:
                min_dist = dist
        return False, min_dist

    def goal_on_path(self, path):
        for idx, state in enumerate(path):
            if self.in_goal_area(state):
                if idx == len(path) - 1:
                    return False
                if self.in_goal_area(path[idx + 1]):  # Check that stay there at least for one more step
                    return True
        return False

    def in_goal_area(self, state):
        """
        Returns true if the state is in the goal region
        """
        current_point = Point(state.x, state.y)
        in_goal = self.goal.contains(current_point)
        return in_goal
      
    def dynamics(self, x0: SpacecraftState, u: SpacecraftCommands) -> SpacecraftState:
        """Dynamics of the spacecraft. Copied over from simulator"""
        acc_lx = apply_acceleration_limits(u.acc_left, self.sc_parameters)
        acc_rx = apply_acceleration_limits(u.acc_right, self.sc_parameters)
        acc_sum = acc_lx + acc_rx
        acc_diff = acc_rx - acc_lx

        vx = x0.vx
        vy = x0.vy
        costh = np.cos(x0.psi)
        sinth = np.sin(x0.psi)
        dx = vx * costh - vy * sinth
        dy = vx * sinth + vy * costh

        ax = acc_sum + x0.vy * x0.dpsi
        ay = -x0.vx * x0.dpsi
        ddpsi = self.sc_geometry.w_half * self.sc_geometry.m / self.sc_geometry.Iz * acc_diff
        ddpsi = apply_rot_speed_constraint(x0.dpsi, ddpsi, self.sc_parameters)
        return SpacecraftState(x=dx, y=dy, psi=x0.dpsi, vx=ax, vy=ay, dpsi=ddpsi)

    def final_path(self, end_node):
        """
        Returns the final path as actions, visited states, and node indices.
        Will return N actions and N+1 states as the initial state is included. This is easily changed if desired.
        """
        states = []
        actions = []
        node_indices = []  # currently not implemented
        node = end_node
        while node.parent is not None:
            states = node.path + states
            actions = node.actions + actions
            node = node.parent

        states = [node.st] + states  # adding the initial position
        return states, actions, self.dynamic_obstacles_state_traj[:len(states)]
