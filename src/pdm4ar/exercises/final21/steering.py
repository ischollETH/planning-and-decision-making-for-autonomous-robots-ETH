from casadi import *
import time


# Problem continous and not already discrete
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
    # classical RK-4  (TODO RK-45)
    k1 = dynamics_continuous(sg, x, u)
    k2 = dynamics_continuous(sg, x + Ts / 2 * k1, u)
    k3 = dynamics_continuous(sg, x + Ts / 2 * k2, u)
    k4 = dynamics_continuous(sg, x + Ts * k3, u)

    return x + Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def dynamics_euler(sg, x, u, Ts):
    # Simple Euler discretization with Ts
    return x + Ts * dynamics_continuous(sg, x, u)


def steer_using_optimization(sg, x0, xfin, N=5, Ts=0.1):
    # TODO check how can speed up ? Put into separate class ? s.t. dont need to init
    # TODO also using casadi function object may help
    # TODO see https://groups.google.com/g/casadi-users/c/UaIuIOEqf6k
    # TODO NLP function evaluation seems to be quite slow (0.3s)
    now = time.time()
    # Using CasADi optimal control
    # Different Methods :
    # Direct single-shooting
    # Direct multiple-shooting <-
    # Direct collocation
    # Problem : How to discretize dynamics ??? Know = 0.01s sampling of simulation uses RK-45
    # Simple Solution : Use euler discretization : problem less precise than RK-45, maybe finer disc. helps
    # Other solutions : Use RK-45, or use one of the above methods
    #
    # Note : sg needs to be set to agent self.sg version for final code :

    # Define Weights
    Plinear = 300.
    Pangular = 80.
    Q = 1.  # Not used
    R = 0.1

    opti = casadi.Opti()
    p_opts = {"expand": True, 'ipopt.print_level':0, 'print_time':0}
    s_opts = {"max_iter": 400,
              } #TODO tweak
    opti.solver("ipopt", p_opts,
                s_opts)
    obj = 0

    X = opti.variable(6, N)
    U = opti.variable(2, N - 1)

    # Slack variable for soft constraint
    slack_linear = opti.variable()
    slack_angular = opti.variable()
    eps_linear = 1
    eps_angular = 1

    slack_linearVx = opti.variable()
    slack_linearVy = opti.variable()
    slack_angularV = opti.variable()
    eps_linearV = 1
    eps_angularV = 1

    # Constraints
    opti.subject_to(X[:, 0] == x0)
    for i in range(N - 1):
        opti.subject_to(X[:, i + 1] == dynamics_rk(sg, X[:, i], U[:, i], Ts))
        opti.subject_to(U[:, i] <= 10)
        opti.subject_to(U[:, i] >= -10)
        obj += R * U[0, i] ** 2 + R * U[1, i] ** 2

    # Terminal cost
    obj += (Plinear * (X[0, N - 1] - xfin[0]) ** 2 + Plinear * (X[1, N - 1] - xfin[1]) ** 2 + Pangular * (
            X[0, N - 1] - xfin[0]))

    # Soft constraints
    opti.subject_to([(X[0, N - 1] - xfin[0]) ** 2 + (X[1, N - 1] - xfin[1]) ** 2 <= slack_linear,
                     (X[2, N - 1] - xfin[2]) ** 2 <= slack_angular])
    # TODO investigate why often infeasible of soft constraint on velocities
    opti.subject_to([fabs(X[3, N - 1]- xfin[3]) <= slack_linearVx, fabs(X[4, N - 1] - xfin[4]) <= slack_linearVy, fabs(X[5, N - 1] - xfin[5]) <= slack_angularV])
    # opti.subject_to([fabs(X[3, N - 1]) <= 0.1, fabs(X[4, N - 1]) <= 0.1, fabs(X[5, N - 1]) <= 0.4])

    # slack cost function (TODO investigate if adding quadratic cost is critical - loose optimality but slightly faster ?)
    obj += (eps_linear * slack_linear**2 + eps_angular * slack_angular**2)
    obj += (eps_linearV * slack_linearVx**2 + eps_linearV * slack_linearVy**2 + eps_angularV * slack_angularV**2)

    opti.minimize(obj)

    # TODO initial guess using heuristic
    # opti.set_initial ..


    success = False
    try:
        sol = opti.solve()
        success = True
    except:
        print("Problem is infeasible")
        # Take suboptimal solution
        return opti.debug.value(U), opti.debug.value(X), success
        # return np.zeros((2,N-1)), np.zeros((2,N)), success
    print(f"Steering done, took : {time.time() - now}")

    return sol.value(U), sol.value(X), success
