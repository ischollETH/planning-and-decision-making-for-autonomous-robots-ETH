from abc import ABC, abstractmethod
from typing import Tuple, Set, Iterator, Sequence
from functools import lru_cache
from itertools import product

import numpy as np
from numpy.typing import NDArray

from pdm4ar.exercises.ex04.structures import Action, Policy, ValueFunc, State, Cell


class GridMdp:
    def __init__(self, grid: NDArray[np.int], gamma: float = 0.7):
        assert len(grid.shape) == 2, "Map is invalid"
        self.grid = grid
        """The map"""
        self.gamma: float = gamma
        """Discount factor"""

    @lru_cache(None)
    def get_feasible_actions(self, state: State) -> Set[Action]:
        """This function produces a set of possible actions given a specific state in the grid"""
        if self.grid[state[0], state[1]] == Cell.GOAL:
            return {Action.STAY}
        else:
            actions: Set[Action] = {Action.NORTH, Action.SOUTH, Action.WEST, Action.EAST}
            if state[0] == self.minmax_x[0]:
                actions.remove(Action.NORTH)
            if state[0] == self.minmax_x[1] - 1:
                actions.remove(Action.SOUTH)
            if state[1] == self.minmax_y[0]:
                actions.remove(Action.WEST)
            if state[1] == self.minmax_y[1] - 1:
                actions.remove(Action.EAST)
            return actions

    def is_terminal_state(self, state: State) -> bool:
        return self.grid[state[0], state[1]] == Cell.GOAL

    def get_transition_prob(self, state: State, action: Action, next_state: State) -> float:
        """Returns P(next_state | state, action)"""
        if next_state == self._dynamics(state, action):
            return 1.0
        return 0.0

    def stage_reward(self, state: State, action: Action) -> float:
        state_cell = self.grid[state]
        if state_cell == Cell.GRASS:
            return -1.0
        elif state_cell == Cell.START:
            return -1.0
        elif state_cell == Cell.SWAMP:
            return -10.0
        elif state_cell == Cell.GOAL:
            return 10.0
        else:
            raise ValueError(f"Unrecognized cell {state_cell}")
        pass

    def is_state_valid(self, state: State) -> bool:
        inside_x = self.minmax_x[0] <= state[0] <= self.minmax_x[1] - 1
        inside_y = self.minmax_y[0] <= state[1] <= self.minmax_y[1] - 1
        return True if inside_x and inside_y else False

    def _dynamics(self, state: State, action: Action) -> State:
        if action == Action.NORTH:
            next_state = state[0] - 1, state[1]
        elif action == Action.SOUTH:
            next_state = state[0] + 1, state[1]
        elif action == Action.WEST:
            next_state = state[0], state[1] - 1
        elif action == Action.EAST:
            next_state = state[0], state[1] + 1
        elif action == Action.STAY:
            next_state = state
        else:
            raise ValueError(f"Unrecognised action: {action} is not valid")

        # Check if we went out
        if not self.is_state_valid(next_state):
            next_state = state
        return next_state

    @property
    def minmax_x(self) -> Tuple[int, int]:
        return 0, self.grid.shape[0]

    @property
    def minmax_y(self) -> Tuple[int, int]:
        return 0, self.grid.shape[1]

    def it_all_states(self) -> Iterator[State]:
        xs, ys = range(*self.minmax_x), range(*self.minmax_y)
        return product(xs, ys)

    @lru_cache(None)
    def it_possible_successors(self, state: State, action: Action) -> Sequence[State]:
        def is_transition_possible(next_state, x=state, a=action):
            p = self.get_transition_prob(x, a, next_state)
            return p > 0

        return tuple(filter(is_transition_possible, list(self.it_all_states())))

class GridMdpSolver(ABC):
    @staticmethod
    @abstractmethod
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        pass
