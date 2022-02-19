from casadi import *
import time

DEBUG = False

# Dynamics model from simulator
def dynamics_continuous(sg, x, u):
    """Kinematic bicycle model, returns state derivative for given control inputs"""
    # x = [x, y, psi, vx, vy, dpsi]
    # u[0] : right
    # u[1] : left

    acc_sum = u[0] + u[1]
    acc_diff = u[0] - u[1]

    vx = x[3]
    vy = x[4]

    costh = cos(x[2])
    sinth = sin(x[2])

    dx = vx * costh - vy * sinth
    dy = vx * sinth + vy * costh

    ax = acc_sum + vy * x[5]
    ay = -vx * x[5]
    ddpsi = sg.w_half * sg.m / sg.Iz * acc_diff  # need to be saturated first
    # ddpsi = apply_rot_speed_constraint(x0.dpsi, ddpsi, self.sp)

    return vertcat(dx, dy, x[5], ax, ay, ddpsi)


def dynamics_rk(sg, x, u, Ts):
    # classical RK-4
    k1 = dynamics_continuous(sg, x, u)
    k2 = dynamics_continuous(sg, x + Ts / 2 * k1, u)
    k3 = dynamics_continuous(sg, x + Ts / 2 * k2, u)
    k4 = dynamics_continuous(sg, x + Ts * k3, u)
    return x + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def dynamics_rk_fine(sg, x, u, ts=0.1, tfine=0.05):
    b = int(ts / tfine)
    for i in range(b):
        x = dynamics_rk(sg, x, u, tfine)
    return x


def dynamics_euler(sg, x, u, Ts):
    # Simple Euler discretization with Ts
    return x + Ts * dynamics_continuous(sg, x, u)


class SteeringOptimizer:
    def __init__(self, N, sg, sp, static_obstacles):
        self.N = N
        self.sg = sg
        self.sp = sp
        self.Ts = 0.1
        sc_border_min_distance = min(self.sg.w_half, self.sg.lf, self.sg.lr)

        # Boundary as convex constraint set
        boundary_obstacle = static_obstacles[0].shape.bounds  # Assuming the boundary is always the first obstacle
        self.x_min = boundary_obstacle[0]
        self.y_min = boundary_obstacle[1]
        self.x_max = boundary_obstacle[2]
        self.y_max = boundary_obstacle[3]

        # Initialize optimization variables
        # Define Weights
        self.Plinear = 100.
        self.Pangular = 80.
        self.R = 80

        self.opti = casadi.Opti()
        p_opts = {
            "expand": True ,'ipopt.print_level': 0, 'print_time': 0}
        s_opts = {"max_iter": 200  # , 'max_cpu_time':0.1,
                  }  # TODO tweak
        self.opti.solver("ipopt", p_opts,
                         s_opts)

        self.X = self.opti.variable(6, N)
        self.U = self.opti.variable(2, N - 1)

        # Slack variable for soft constraint
        # self.slack_linear = self.opti.variable()
        # self.slack_angular = self.opti.variable()
        # self.eps_linear = 1
        # self.eps_angular = 1
        #
        # self.slack_linearVx = self.opti.variable()
        # self.slack_linearVy = self.opti.variable()
        # self.slack_angularV = self.opti.variable()
        # self.eps_linearV = 1
        # self.eps_angularV = 1

        self.obj = 0

        # static constraints/cost
        for i in range(self.N - 1):
            self.opti.subject_to(self.X[:, i + 1] == dynamics_rk(self.sg, self.X[:, i], self.U[:, i], self.Ts))
            # self.opti.subject_to(self.X[:, i + 1] == dynamics_rk_fine(self.sg, self.X[:, i], self.U[:, i]))
            self.opti.subject_to(self.U[:, i] <= self.sp.acc_limits[1] - 0.0001)
            self.opti.subject_to(self.U[:, i] >= self.sp.acc_limits[0] + 0.0001)
            self.obj += self.R * self.U[0, i] ** 2 + self.R * self.U[1, i] ** 2

            # Force states within boundary
            self.opti.subject_to(self.X[0, i] <= self.x_max - sc_border_min_distance)
            self.opti.subject_to(self.X[0, i] >= self.x_min + sc_border_min_distance)
            self.opti.subject_to(self.X[1, i] <= self.y_max - sc_border_min_distance)
            self.opti.subject_to(self.X[1, i] >= self.y_min + sc_border_min_distance)

        # slack cost function
        # self.obj += (self.eps_linear * self.slack_linear ** 2 + self.eps_angular * self.slack_angular ** 2)
        # self.obj += (
        #        self.eps_linearV * self.slack_linearVx ** 2 + self.eps_linearV * self.slack_linearVy ** 2 + self.eps_angularV * self.slack_angularV ** 2)

    def steer(self, initial_state, goal_state):

        temp_opti = self.opti.copy()
        temp_obj = self.obj
        temp_opti.subject_to(self.X[:, 0] == initial_state)

        # # Terminal cost - position
        # temp_obj += self.Plinear * (self.X[0, self.N - 1] - goal_state[0]) ** 2 + self.Plinear * (
        #         self.X[1, self.N - 1] - goal_state[1]) ** 2 + self.Pangular * (
        #                      self.X[2, self.N - 1] - goal_state[2])**2
        #
        # # Terminal cost - velocity
        # temp_obj += 0.1 * (self.Plinear * (self.X[3, self.N - 1] - goal_state[3]) ** 2 + self.Plinear * (
        #         self.X[4, self.N - 1] - goal_state[4]) ** 2 + self.Pangular * (
        #                      self.X[5, self.N - 1] - goal_state[5])**2)


        # cost on goal deviation as stage cost (includes terminal cost)
        for i in range(self.N):
            # stage cost position/angle
            temp_obj += 1*(self.Plinear * (self.X[0, i] - goal_state[0]) ** 2 + self.Plinear * (
                    self.X[1, i] - goal_state[1]) ** 2 + self.Pangular * (
                                        self.X[2, i] - goal_state[2])**2)

            # stage cost - velocity -> care less about
            temp_obj += 0.1 * (self.Plinear * (self.X[3, i] - goal_state[3]) ** 2 + self.Plinear * (
                    self.X[4, i] - goal_state[4]) ** 2 + self.Pangular * (
                                       self.X[5, i] - goal_state[5]) ** 2)

        # Soft constraints
        # temp_opti.subject_to([(self.X[0, self.N - 1] - goal_state[0]) ** 2 + (
        #         self.X[1, self.N - 1] - goal_state[1]) ** 2 <= self.slack_linear,
        #                       (self.X[2, self.N - 1] - goal_state[2]) ** 2 <= self.slack_angular])
        # temp_opti.subject_to([fabs(self.X[3, self.N - 1] - goal_state[3]) <= self.slack_linearVx,
        #                       fabs(self.X[4, self.N - 1] - goal_state[4]) <= self.slack_linearVy,
        #                       fabs(self.X[5, self.N - 1] - goal_state[5]) <= self.slack_angularV])
        # temp_opti.subject_to([fabs(X[3, N - 1]) <= 0.1, fabs(X[4, N - 1]) <= 0.1, fabs(X[5, N - 1]) <= 0.4])

        now = time.time()
        temp_opti.minimize(temp_obj)

        success = False
        try:
            sol = temp_opti.solve()
            success = True
        except:
            if DEBUG:
                print(f"Problem is infeasible, took : {time.time() - now}")
            # input()
            # Take suboptimal solution
            return temp_opti.debug.value(self.U), temp_opti.debug.value(self.X), success
            # return np.zeros((2,N-1)), np.zeros((2,N)), success
        if DEBUG:
            print(f"Steering done, took : {time.time() - now}")

        return sol.value(self.U), sol.value(self.X), success
