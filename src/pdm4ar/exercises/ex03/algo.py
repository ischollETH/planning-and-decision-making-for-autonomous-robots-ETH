from abc import ABC, abstractmethod
from functools import partial
from queue import PriorityQueue
from typing import Optional, List, MutableMapping

import numpy as np

from pdm4ar.exercises.ex02.structures import X
from pdm4ar.exercises.ex03.structures import WeightedGraph


class InformedGraphSearch(ABC):
    queue: PriorityQueue
    parents: MutableMapping[X, Optional[X]]
    cost2reach: MutableMapping[X, float]

    @abstractmethod
    def path(self, graph: WeightedGraph, start: X, goal: X) -> Optional[List[X]]:
        pass

    def recursive_path_from_parents(self, end: X) -> List[X]:
        path = [end, ]
        while self.parents[end] is not None:
            path.append(self.parents[end])
            end = self.parents[end]
        return list(reversed(path))


class UniformCostSearch(InformedGraphSearch):
    def path(self, graph: WeightedGraph, start: X, goal: X) -> Optional[List[X]]:
        self.queue = PriorityQueue()
        self.queue.put((0, start))
        self.cost2reach = {start: 0}
        self.parents = {start: None}
        while not self.queue.empty():
            cost_candidate, candidate = self.queue.get()
            if candidate == goal:
                return self.recursive_path_from_parents(goal)
            for i in graph.adj_list[candidate]:
                new_cost = cost_candidate + graph.get_weight(candidate, i)
                curr_cost = self.cost2reach.get(i)
                if curr_cost is None or new_cost < curr_cost:
                    self.cost2reach[i] = new_cost
                    self.parents[i] = candidate
                    self.queue.put((new_cost, i))
        return None


class GreedyBestFirst(InformedGraphSearch):
    def path(self, graph: WeightedGraph, start: X, goal: X) -> Optional[List[X]]:
        euclidean_heu = partial(euclidean_heuristic, goal=goal, wG=graph)
        self.queue = PriorityQueue()
        self.queue.put((euclidean_heu(start), start))
        self.parents = {start: None}
        while not self.queue.empty():
            cost_candidate, candidate = self.queue.get()
            if candidate == goal:
                return self.recursive_path_from_parents(goal)
            for i in graph.adj_list[candidate]:
                heu = euclidean_heu(i)
                if i not in self.parents:
                    self.parents[i] = candidate
                    self.queue.put((heu, i))
        return None


class Astar(InformedGraphSearch):
    def path(self, graph: WeightedGraph, start: X, goal: X) -> Optional[List[X]]:
        euclidean_heu = partial(euclidean_heuristic, goal=goal, wG=graph)
        self.queue = PriorityQueue()
        self.queue.put((0, start))
        self.parents = {start: None}
        self.cost2reach = {start: 0}
        while not self.queue.empty():
            _, candidate = self.queue.get()
            if candidate == goal:
                return self.recursive_path_from_parents(goal)
            for successor in graph.adj_list[candidate]:
                new_cost2succ = self.cost2reach[candidate] + graph.get_weight(candidate, successor)
                curr_cost2succ = self.cost2reach.get(successor)
                if curr_cost2succ is None or new_cost2succ < curr_cost2succ:
                    self.cost2reach[successor] = new_cost2succ
                    self.parents[successor] = candidate
                    priority = new_cost2succ + euclidean_heu(successor)
                    self.queue.put((priority, successor))
        return None


def euclidean_heuristic(node: X, goal: X, wG: WeightedGraph) -> float:
    n_lon_lat = np.array([wG.get_node_attribute(node, "x"), wG.get_node_attribute(node, "y")])
    g_lon_lat = np.array([wG.get_node_attribute(goal, "x"), wG.get_node_attribute(goal, "y")])
    return np.linalg.norm(n_lon_lat - g_lon_lat)


def compute_path_cost(wG: WeightedGraph, path: List[X]):
    """A utility function to compute the cumulative cost along a path"""
    total: float = 0
    for i in range(1, len(path)):
        inc = wG.get_weight(path[i - 1], path[i])
        total += inc
    return total
