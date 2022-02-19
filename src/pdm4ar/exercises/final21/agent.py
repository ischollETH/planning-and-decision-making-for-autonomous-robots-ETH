from typing import Sequence

import numpy as np
from dg_commons import PlayerName
from dg_commons.planning import PolygonGoal
from dg_commons.sim import SimObservations
from dg_commons.sim.agents import Agent
from dg_commons.sim.models.obstacles import StaticObstacle
from dg_commons.sim.models.spacecraft import SpacecraftCommands, SpacecraftState
from dg_commons.sim.models.spacecraft_structures import SpacecraftGeometry, SpacecraftParameters
from pdm4ar.exercises.final21.rrt import RRT
import matplotlib
from matplotlib import pyplot as plt
from dg_commons.maps.shapely_viz import ShapelyViz


class Pdm4arAgent(Agent):
    """This is the PDM4AR agent.
    Do NOT modify the naming of the existing methods and the input/output types.
    Feel free to add additional methods, objects and functions that help you to solve the task"""

    def __init__(self,
                 goal: PolygonGoal,
                 static_obstacles: Sequence[StaticObstacle],
                 sg: SpacecraftGeometry,
                 sp: SpacecraftParameters):
        self.goal = goal
        self.static_obstacles = static_obstacles
        self.sg = sg
        self.sp = sp
        self.name = None
        self.RRT = None
        self.dt = 0.1  # We assume this, but i guess it could change.
        self.recompute_RRT = True
        self.planned_states = []  # maybe it makes more sense to have these at the RRT class
        self.true_states = []
        self.planned_actions = []
        self.planned_dyn_obstacles = []
        self.true_dyn_obstacles = []

    def on_episode_init(self, my_name: PlayerName):
        self.name = my_name

    def get_commands(self, sim_obs: SimObservations) -> SpacecraftCommands:
        """ This method is called by the simulator at each time step.

        This is how you can get your current state from the observations:
        my_current_state: SpacecraftState = sim_obs.players[self.name].state

        :param sim_obs:
        :return:
        """
        time_step = float(sim_obs.time) / self.dt  # not yielding the right result
        time_step = round(time_step)

        self.true_states.append(sim_obs.players[self.name].state)
        cur_dyn_obs = [value for key, value in sim_obs.players.items() if key not in [self.name]]
        self.true_dyn_obstacles.append(cur_dyn_obs)

        # check if the dyn_obstacle has deviated or no:
        # for act_obst in self.true_dyn_obstacles[-1]:
        #     act_state = act_obst.state
        if self.RRT is not None:
            if all(self.true_dyn_obstacles):
                for est_state, act_obst in zip(self.planned_dyn_obstacles[time_step],
                                               self.true_dyn_obstacles[time_step]):
                    act_state = act_obst.state
                    dyn_obj_diff = act_state - est_state
                    dyn_obj_dist = np.sqrt(dyn_obj_diff.x**2 + dyn_obj_diff.y**2)

                # # brute force approach for deciding whether to replan
                # if dyn_obj_dist > 0.01:
                #     self.RRT.reset_dyn_obs_to_time(float(sim_obs.time), cur_dyn_obs)
                #     self.recompute_RRT = True

                # 'smart' approach for deciding whether to replan based one whether a collision appears now
                if dyn_obj_dist > 0.01:
                    self.RRT.reset_dyn_obs_to_time(float(sim_obs.time), cur_dyn_obs)
                    collision = self.RRT.check_collision_path_dyn(self.planned_states, (len(self.planned_states)-1)*self.dt)
                    self.planned_dyn_obstacles = self.RRT.dynamic_obstacles_state_traj
                    if collision:
                        self.recompute_RRT = True

            state_diff = self.true_states[time_step]-self.planned_states[time_step]
            if max(abs(state_diff.x), abs(state_diff.y)) > 0.05 or abs(state_diff.psi) > 0.025:
                self.recompute_RRT = True

        # initialise the RRT object and run planning
        if self.recompute_RRT:
            start = sim_obs.players[self.name].state
            dyn_obs = [value for key, value in sim_obs.players.items() if key not in [self.name]]
            self.RRT = RRT(start, self.goal, self.static_obstacles, dyn_obs, self.dt, self.sp, self.sg)

            # Run the planning. Comment this line if you rather want to test methods individually
            print("Start computing trajectory using RRT")
            planned_states, planned_actions, planned_dyn_obstacles = self.RRT.plan()
            self.planned_states = self.planned_states[:time_step] + planned_states
            self.planned_actions = self.planned_actions[:time_step] + planned_actions
            self.planned_dyn_obstacles = self.planned_dyn_obstacles[:time_step] + planned_dyn_obstacles
            print("Plan computed")
            # self.true_states = [self.true_states[-1]]
            # self.true_dyn_obstacles = [self.true_dyn_obstacles[-1]]
            self.recompute_RRT = False

        # self.perform_self_tests()
        action = self.planned_actions[time_step]
        # if (time_step > 15 and time_step % 15 == 0) or self.RRT.in_goal_area(self.planned_states[time_step+1]):
        #     # self.RRT.reset_dyn_obs_to_time(float(sim_obs.time), cur_dyn_obs)
        #     # self.RRT.check_collision_path_dyn(self.planned_states, (len(self.planned_states)-1)*self.dt)
        #     self.draw_state_trajectory()
        return action  # SpacecraftCommands(acc_left=1, acc_right=1)

    def draw_state_trajectory(self):
        matplotlib.use('TkAgg')
        plt.close('all')
        plt.figure()
        ax = plt.gca()
        shapely_viz = ShapelyViz(ax)
        true_path_len = len(self.true_states)
        true_path_x = [st.x for st in self.true_states]
        true_path_y = [st.y for st in self.true_states]
        planned_path_x = [st.x for st in self.planned_states[:true_path_len]]
        planned_path_y = [st.y for st in self.planned_states[:true_path_len]]
        plt.plot(true_path_x, true_path_y)
        plt.plot(planned_path_x, planned_path_y)

        for s_obstacle in self.static_obstacles:
            shapely_viz.add_shape(s_obstacle.shape, color=s_obstacle.geometry.color)
        shapely_viz.add_shape(self.goal.get_plottable_geometry(), color="orange", alpha=0.5)
        # for obstacles_polygons in self.RRT.dynamic_obstacles_polygon_traj[:true_path_len:5]:
        #     for obst_poly in obstacles_polygons:
        #         shapely_viz.add_shape(obst_poly, color="green", alpha=0.3)
        for obstacles in self.true_dyn_obstacles[::5]:
            for obst in obstacles:
                shapely_viz.add_shape(obst.occupancy, color="blue", alpha=0.3)
                x = obst.state.x
                y = obst.state.y
                psi = obst.state.psi
                origin = np.array([x, y])
                psi_x = np.cos(psi)*3
                psi_y = np.sin(psi)*3
                plt.quiver(*origin, psi_x, psi_y)
                plt.quiver(*origin, psi_x, psi_y)
        plt.show()

    def perform_self_tests(self):
        # testing of individual RRT-methods.
        # anything below here has simply been used for verifying individual RRT-methods. Can be ignored/deleted.
        action_seq_np = np.array([[-10, 0, 2, 10, 5, 2, -6, -8, -10, -10, -10, 0, 2, 10, 5, 2, -6, -8, -10, -10, 0, 0.5,
                                   -0.5, 0, 4, 10, 10, 5, -5, 0, 0, 0.5, -0.5, 0, 4, 10, 10, 5, -5, 0],
                                  [0, 0.5, -0.5, 0, 4, 10, 10, 5, -5, 0, 0, 0.5, -0.5, 0, 4, 10, 10, 5, -5, 0, 0, 0.5,
                                   -0.5, 0, 4, 10, 10, 5, -5, 0, 0, 0.5, -0.5, 0, 4, 10, 10, 5, -5, 0]])
        action_seq = []

        st = SpacecraftState(x=40, y=80, psi=np.pi / 2, vx=0, vy=0, dpsi=0)

        # tests goal area function
        tmp = self.RRT.in_goal_area(st)

        # tests collision detection
        collision, dist = self.RRT.check_collision_state(st)

        # tests steering
        test_state = SpacecraftState(x=20, y=20, psi=0, vx=2, vy=1.5, dpsi=2)
        target_state = SpacecraftState(x=10, y=30, psi=0, vx=0, vy=0, dpsi=0)
        actions = self.RRT.steer(test_state, target_state, 100)
        path = self.RRT.simulate(test_state, actions, time_div=1)

        # plot results of steering
        matplotlib.use('TkAgg')
        x_coords = [st.x for st in path]
        y_coords = [st.y for st in path]
        plt.figure()
        plt.plot(x_coords, y_coords)
        psi_coords = [st.psi for st in path]
        psi_x = np.cos(psi_coords)
        psi_y = np.sin(psi_coords)
        origin = np.array([x_coords, y_coords])
        plt.quiver(*origin, psi_x, psi_y)
        plt.quiver(*origin, psi_x, psi_y)
        plt.figure()
        plt.plot(psi_coords)
        plt.show()
