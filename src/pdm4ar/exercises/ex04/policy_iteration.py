from typing import Tuple
from copy import copy

import numpy as np

from pdm4ar.exercises.ex04.mdp import GridMdp, GridMdpSolver
from pdm4ar.exercises.ex04.structures import ValueFunc, Policy
from pdm4ar.exercises_def.ex04.utils import time_function
from pdm4ar.exercises.ex04.value_iteration import get_optimal_action, get_action_value


def policy_evaluation(mdp: GridMdp, policy: Policy, value_func: ValueFunc) -> ValueFunc:
    """
    Policy evaluation
    Computes V^{pi}(s) for all states given a specific policy pi.
    """
    n_states = len(list(mdp.it_all_states()))
    value_policy = np.zeros_like(value_func)
    a, b = np.zeros((n_states, n_states)), np.zeros(n_states)
    state2idx = dict(zip(mdp.it_all_states(), range(n_states)))
    for i, x in enumerate(mdp.it_all_states()):
        value_policy[x] = get_action_value(mdp, value_func, x, policy[x])
        for next_x in mdp.it_possible_successors(x, policy[x]):
            a[i][state2idx[next_x]] = -mdp.gamma * mdp.get_transition_prob(x, policy[x], next_x)
        b[i] = mdp.stage_reward(x, policy[x])
    a += np.eye(n_states)
    value_policy = np.linalg.solve(a, b)
    return value_policy.reshape(mdp.minmax_x[1], mdp.minmax_y[1])


def policy_improvement(mdp: GridMdp, value_func: ValueFunc) -> Policy:
    """
    Computes the new policy
    """
    policy = np.zeros_like(value_func).astype(int)
    for x in mdp.it_all_states():
        policy[x] = get_optimal_action(mdp, value_func, x)

    return policy


class PolicyIteration(GridMdpSolver):
    @staticmethod
    @time_function
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        tol: float = 1e-7
        value_func = np.zeros_like(grid_mdp.grid).astype(float)
        policy = np.zeros_like(grid_mdp.grid).astype(int)
        policy_converged = False
        while not policy_converged:
            vfunc_max_it: int = 200
            diff = 1
            while diff > tol and vfunc_max_it > 0:
                old_value_func = copy(value_func)
                value_func = policy_evaluation(grid_mdp, policy, old_value_func)
                diff = np.linalg.norm(abs(old_value_func - value_func), ord=np.inf)
                vfunc_max_it -= 1

            old_policy = copy(policy)
            policy = policy_improvement(grid_mdp, value_func)
            policy_converged = np.alltrue(np.equal(old_policy, policy))
        return value_func, policy
