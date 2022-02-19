from typing import TypeVar, Set, Mapping, Tuple

X = TypeVar("X")

AdjacencyList = Mapping[X, Set[X]]
"""Simple type alias"""
Query = Tuple[X, X]
"""Simple type alias with start and goal"""
