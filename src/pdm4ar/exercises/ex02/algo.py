from abc import abstractmethod, ABC
from collections import deque
from typing import List, Optional, MutableMapping


from pdm4ar.exercises.ex02.structures import AdjacencyList, X


class GraphSearch(ABC):
    Q: deque
    parents: MutableMapping[X, Optional[X]]


    @abstractmethod
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Optional[List[X]]:
        """
        :param graph: The given graph as an adjacency list
        :param start: The initial state (i.e. a node)
        :param goal: The goal state (i.e. a node)
        :return: The path from start to goal as a Sequence of states, None if a path does not exist
        """
        pass

    def recursive_path_from_parents(self, end: X) -> List[X]:
        path = [end, ]
        while self.parents[end] is not None:
            path.append(self.parents[end])
            end = self.parents[end]
        return list(reversed(path))


class DepthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Optional[List[X]]:
        self.Q = deque([start, ])
        self.parents = {start: None}
        while len(self.Q):
            s = self.Q.popleft()
            if s == goal:
                return self.recursive_path_from_parents(goal)
            for i in graph[s]:
                if i not in self.parents:
                    self.parents[i] = s
                    self.Q.appendleft(i)
        return None


class BreadthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Optional[List[X]]:
        self.Q = deque([start, ])
        self.parents = {start: None}
        while len(self.Q):
            s = self.Q.popleft()
            if s == goal:
                return self.recursive_path_from_parents(goal)
            for i in graph[s]:
                if i not in self.parents:
                    self.parents[i] = s
                    self.Q.append(i)
        return None


class IterativeDeepening(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Optional[List[X]]:
        d, m = 0, len(graph)
        found: bool = False
        while not found and d < m:
            self.parents = {start: None}
            found = self.dfs(graph, start, goal, d)
            d += 1
        return self.recursive_path_from_parents(goal) if found else None

    def dfs(self, graph: AdjacencyList, start: X, goal: X, depth: int) -> bool:
        """Depth first search"""
        if depth == 0:
            return True if start == goal else False
        else:
            for i in graph[start]:
                if i not in self.parents:
                    self.parents[i] = start
                    if self.dfs(graph, i, goal, depth - 1):
                        return True
            return False
