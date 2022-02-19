from copy import deepcopy
import matplotlib
from dg_commons.maps.shapely_viz import ShapelyViz
from dg_commons.sim.models.model_utils import apply_acceleration_limits, apply_rot_speed_constraint
from dg_commons.sim.models.spacecraft import SpacecraftState, SpacecraftModel
from dg_commons.sim.simulator_visualisation import ZOrders
from scipy.integrate import solve_ivp

from pdm4ar.exercises.final21.debug_tools import plot_trajectory, context_generator
from pdm4ar.exercises.final21.steering import steer_using_optimization
from pdm4ar.exercises.final21.steering_v2 import SteeringOptimizer
#from pdm4ar.exercises_def import get_sim_context_static

from dg_commons.sim.simulator import Simulator
import matplotlib.pyplot as plt

from casadi import *


def dynamics_with_limits(sg, sp, x, u):
    """Kinematic bicycle model, returns state derivative for given control inputs"""
    acc_lx = apply_acceleration_limits(u[1], sp)
    acc_rx = apply_acceleration_limits(u[0], sp)
    acc_sum = acc_lx + acc_rx
    acc_diff = acc_rx - acc_lx

    vx = x[3]
    vy = x[4]

    costh = cos(x[2])
    sinth = sin(x[2])

    dx = vx * costh - vy * sinth
    dy = vx * sinth + vy * costh

    ax = acc_sum + vy * x[5]
    ay = -vx * x[5]
    ddpsi = sg.w_half * sg.m / sg.Iz * acc_diff
    ddpsi = apply_rot_speed_constraint(x[5], ddpsi, sp)
    return np.array([dx, dy, x[5], ax, ay, ddpsi])


def steering_ilqr(sg, sp, x0, xgoal, N=5, dt=0.1):
    """ Use iLQR (LQR for nonlinear models to come up with a path
    e.g. use https://github.com/anassinator/ilqr
    """
    pass


def steering_pid(sg, sp, x0, xgoal, N=5, dt=0.1):
    """ PID + simulation """
    # TODO need to change gains currently wrong
    kp = np.ones((2, 3))
    kd = np.ones((2, 3))

    U = np.zeros((2, N - 1))
    X = np.zeros((6, N))

    X[:, 0] = x0
    t = 0

    def _dynamics(t, y):
        dx = dynamics_with_limits(sg, sp, x=y[:6], u=y[6:])
        du = np.zeros((2, 1)).flatten()
        return np.concatenate([dx, du])

    for i in range(N - 1):
        t = t + dt
        # TODO check add possible d, i, element
        U[:, i] = -kp @ (X[:3, i] - xgoal[:3])  # - kd @ (X[3:, i] ) #- xgoal[3:])
        y = np.concatenate([X[:, i], U[:, i]])
        sol = solve_ivp(fun=_dynamics, t_span=(t, t + dt), y0=y)
        y = sol.y[:, -1]
        X[:, i + 1] = y[:6]
    return U, X, True


def fwd_simulation(x0, U, N, Tsim, Tcmd):
    diff_steps = int(Tcmd / Tsim)

    X_full_t = np.zeros((6, N * diff_steps))
    X_reduced_t = np.zeros((6, N))

    X_full_t[:, 0] = x0
    X_reduced_t[:, 0] = x0
    t = 0
    dt = Tsim

    def _dynamics(t, y):
        dx = dynamics_with_limits(sg, sp, x=y[:6], u=y[6:])
        du = np.zeros((2, 1)).flatten()
        return np.concatenate([dx, du])

    l = -1
    for i in range((N - 1) * diff_steps):
        if i % diff_steps == 0:
            l += 1

        t = t + dt
        y = np.concatenate([X_full_t[:, i], U[:, l]])
        sol = solve_ivp(fun=_dynamics, t_span=(0, dt), y0=y)  # Autonomous system
        y = sol.y[:, -1]
        X_full_t[:, i + 1] = y[:6]

        if i % diff_steps == diff_steps-1:
            X_reduced_t[:, l+1] = y[:6]

    return X_reduced_t


# Optimization Test - Doesn't use simulator
if False and __name__ == '__main__':
    matplotlib.use('TkAgg')
    x0 = np.array([0.1, 0.1, -0.8, 0, 0, 0])
    xfin = np.array([5, 6,0, 2, 0, 0])
    N = 50
    T_cmd = 0.1
    T_sim = 0.01

    space_craft_x0 = SpacecraftState.from_array(x0)
    model = SpacecraftModel.default(space_craft_x0)
    sg = deepcopy(model.get_geometry())
    sp = deepcopy(model.sp)

    # U, X, success = steer_using_optimization(sg, x0, xfin, N, T_cmd)
    steering = SteeringOptimizer(N, sg)
    U, X, success = steering.steer(x0, xfin)

    # U, X, success = steering_pid(sg, sp, x0, xfin, N)

    # print(U)
    print(f'initial position: {X[1:4, 0]}')
    print(f'terminal velocities (linear) : {X[3:5, -1]}, (angular) : {X[5, -1]}')
    print(f'deviation from goal {np.abs(X[:, -1] - np.abs(xfin))}')

    t = np.arange(0, N)

    # Plot steering inputs
    fig_input, ax_input = plt.subplots()
    ax_input.plot(t[:-1], U[0, :], label='U0 - right')
    ax_input.plot(t[:-1], U[1, :], label='U1 - left')
    ax_input.legend()


    # Solve real simulation
    if True:
        X_sim = fwd_simulation(x0, U, N, T_sim, T_cmd)
        print(X_sim.shape)
        print(X.shape)

        fig_traj, ax_traj = plot_trajectory(X_sim[0, :], X_sim[1, :], X_sim[2, :], t, 'Simulated')

        # Overlay
        ax_traj.scatter(X[0, :], X[1, :], label='Planned', marker='^', color='red')
        ax_traj.legend()

        # Plot per -axis only
        fig_x, ax_x = plt.subplots(3)
        fig_x.suptitle(f"Trajectories Simulated vs Planned : {T_sim, T_cmd}s ")
        # x axis
        ax_x[0].plot(t, X[0, :], label='Steering', color='k')
        ax_x[0].plot(t, X_sim[0, :], label='Simulated', color='tab:cyan')
        ax_x[0].set_ylabel('x')
        ax_x[0].legend()

        ax_x[0].tick_params(axis='y', labelcolor='tab:blue')
        ax2 = ax_x[0].twinx()  # instantiate a second axes that shares the same x-axis
        ax2.plot(t, np.abs(X[0, :]-X_sim[0,:]), color='tab:red')
        ax2.set_ylabel('abs. error')
        ax2.tick_params(axis='y', labelcolor='tab:red')

        # y-axis
        ax_x[1].plot(t, X[1, :], label='Steering', color='k')
        ax_x[1].plot(t, X_sim[1, :], label='Simulated',color='tab:cyan')
        ax_x[1].set_ylabel('y')
        ax_x[1].legend()

        ax_x[1].tick_params(axis='y', labelcolor='tab:blue')
        ax2 = ax_x[1].twinx()  # instantiate a second axes that shares the same x-axis
        ax2.plot(t, np.abs(X[1, :]-X_sim[1,:]), color='tab:red')
        ax2.set_ylabel('abs. error')
        ax2.tick_params(axis='y', labelcolor='tab:red')

        # psi axis
        ax_x[2].plot(t, X[2, :], label='Steering', color='k')
        ax_x[2].plot(t, X_sim[2, :], label='Simulated', color='tab:cyan')
        ax_x[2].set_ylabel('psi')
        ax_x[2].legend()

        ax_x[2].tick_params(axis='y', labelcolor='tab:blue')
        ax2 = ax_x[2].twinx()  # instantiate a second axes that shares the same x-axis
        ax2.plot(t, np.abs(X[2, :]-X_sim[2,:]), color='tab:red')
        ax2.set_ylabel('abs. error')
        ax2.tick_params(axis='y', labelcolor='tab:red')

        print(f'Error steering vs simulation :{np.abs(X-X_sim).max(1)}')


    plt.show()

# Simulation Test
if True and __name__ == '__main__':
    seed = 20
    # dgScenario, goal, x0 = get_dgscenario(seed)

    # Define sim context - uses PDM4AR Agent
    # sim_context = get_sim_context_dynamic(seed)
    # sim_context = get_sim_context_static(seed)
    sim_context = context_generator(seed, noObstacles=False)

    # Setup simulation
    sim = Simulator()

    # Run simulation
    sim.run(sim_context)

    # Get planned states
    #print(sim_context.players['PDM4AR'].planned_states)
    planned_states = sim_context.players['PDM4AR'].planned_states
    planned_x_trajectory = [state.x for state in planned_states]
    planned_y_trajectory = [state.y for state in planned_states]
    planned_psi_trajectory = [state.psi for state in planned_states]

    planned_actions = sim_context.players['PDM4AR'].planned_actions
    action_left = [a.acc_left for a in planned_actions]
    action_right = [a.acc_right for a in planned_actions]


    # Extract logging information
    print(sim_context.log.keys())
    playerlog = sim_context.log['PDM4AR']
    # print(playerlog.states)
    # print(playerlog.actions)
    # print(playerlog.extra)
    # print(playerlog.info)

    sampled_seq = playerlog.states
    timestamps = sampled_seq.timestamps
    values = sampled_seq.values

    print(len(timestamps))
    print(f' Initial values : {values[0]}')

    x_trajectory = [state.x for state in values]
    y_trajectory = [state.y for state in values]
    psi_trajectory = [state.psi for state in values]

    time = np.asarray(timestamps)
    time = time.astype('float')

    # Plot trajectory
    fig, ax = plot_trajectory(x_trajectory, y_trajectory, psi=psi_trajectory, time=time)

    # Plot scenario
    # matplotlib.use('TkAgg')
    dg_scenario = sim_context.dg_scenario
    # ax = plt.gca()
    shapely_viz = ShapelyViz(ax)

    for s_obstacle in dg_scenario.static_obstacles.values():
        shapely_viz.add_shape(s_obstacle.shape, color=s_obstacle.geometry.color, zorder=ZOrders.ENV_OBSTACLE)
    # shapely_viz.add_shape(goal.get_plottable_geometry(), color="orange", zorder=ZOrders.GOAL, alpha=0.5)
    ax = shapely_viz.ax
    ax.autoscale()
    ax.set_facecolor('w')
    ax.set_aspect("equal")


    #overlay planned trajectory
    ax.plot(planned_x_trajectory, planned_y_trajectory)

    # plot control  actions
    fig, ax = plt.subplots()
    time = range(len(action_left))
    ax.plot(time, action_left)
    ax.plot(time,action_right)

    plt.show()
