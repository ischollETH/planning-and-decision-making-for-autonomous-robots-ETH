from typing import Tuple
from copy import copy

import numpy as np

from pdm4ar.exercises.ex04.mdp import GridMdp, GridMdpSolver
from pdm4ar.exercises.ex04.structures import Action, ValueFunc, Policy, State
from pdm4ar.exercises_def.ex04.utils import time_function


def get_action_value(mdp: GridMdp, value_func: ValueFunc, state: State, action: Action) -> float:
    """Computes Q(s,a)"""
    possible_successors = mdp.it_possible_successors(state, action)
    q = [mdp.get_transition_prob(state, action, new_state) * value_func[new_state] for new_state in possible_successors]
    return mdp.stage_reward(state, action) + mdp.gamma * sum(q)


def get_new_state_value(mdp: GridMdp, value_func: ValueFunc, state: State) -> float:
    """Computes V(s)"""
    v = [get_action_value(mdp, value_func, state, a) for a in mdp.get_feasible_actions(state)]
    return max(v)


class ValueIteration(GridMdpSolver):
    @staticmethod
    @time_function
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        tol: float = 1e-7
        max_iterations: int = 1000
        value_func = np.zeros_like(grid_mdp.grid).astype(float)
        policy = np.zeros_like(grid_mdp.grid).astype(int)
        diff = 1
        while diff > tol and max_iterations > 0:
            old_value_func = copy(value_func)
            for i, j in grid_mdp.it_all_states():
                value_func[i, j] = get_new_state_value(grid_mdp, value_func, (i, j))

            diff = np.linalg.norm(abs(old_value_func - value_func), ord=np.inf)
            max_iterations -= 1
        # here compute policy from the value function
        for i, j in grid_mdp.it_all_states():
            policy[i, j] = get_optimal_action(grid_mdp, value_func, (i, j))
        return value_func, policy


def get_optimal_action(grid_mdp: GridMdp, value_func: ValueFunc, state: State) -> Action:
    """Find optimal action using the above formula"""
    if grid_mdp.is_terminal_state(state):
        return Action.STAY

    optimal_actions = {
        a: get_action_value(grid_mdp, value_func, state, a) for a in grid_mdp.get_feasible_actions(state)
    }
    optimal_a = max(optimal_actions, key=optimal_actions.get)
    return optimal_a
