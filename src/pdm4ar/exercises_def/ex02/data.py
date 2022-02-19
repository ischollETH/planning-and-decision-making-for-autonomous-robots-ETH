from dataclasses import dataclass
from itertools import product
from random import sample
from typing import Set, List

import networkx as nx
from networkx import random_geometric_graph, DiGraph

from pdm4ar.exercises.ex02.structures import AdjacencyList, Query


@dataclass
class GraphSearchProblem:
    graph: AdjacencyList
    queries: Set[Query]


def networkx_2_adjacencylist(nxgraph: DiGraph) -> AdjacencyList:
    adj_list = dict()
    atlas = nxgraph.adj._atlas
    for n in atlas.keys():
        adj_list[n] = set(atlas[n].keys())
    return adj_list


def queries_from_adjacency(adj_list: AdjacencyList, n: int) -> Set[Query]:
    graph_queries = set()
    nodes = list(adj_list.keys())
    for _ in range(n):
        query_pair = sample(nodes, 2)
        graph_queries.add(tuple(query_pair))
    return graph_queries


def get_graph_search_problems() -> List[GraphSearchProblem]:
    graphsearch_prob = list()
    # test graph 1
    easy01: AdjacencyList = {"A": {"B", "C"}, "B": {"C", "D"}, "C": {"D"}, "D": {"C"}, "E": {"F"}, "F": {"C"}}
    easy01_queries = {("A", "D"), ("B", "F"), ("F", "A"), ("F", "F"), ("E", "D")}
    graphsearch_prob.append(GraphSearchProblem(graph=easy01, queries=easy01_queries))
    # test graph 2
    size_g2 = 100
    graph02_nx = random_geometric_graph(size_g2, 0.125)
    graph02: AdjacencyList = networkx_2_adjacencylist(graph02_nx)
    graph02_queries = queries_from_adjacency(graph02, 3)

    graphsearch_prob.append(GraphSearchProblem(graph=graph02, queries=graph02_queries))

    # test graph three
    branching = 3
    height = 4
    graph03_nx = nx.balanced_tree(branching, height)
    graph03_nx = nx.bfs_tree(graph03_nx, 0)
    graph03: AdjacencyList = networkx_2_adjacencylist(graph03_nx)
    goals = sample(list(range(len(graph03))), 5)
    graph03_queries = tuple(
        product(
            [
                0,
            ],
            goals,
        )
    )
    graphsearch_prob.append(GraphSearchProblem(graph=graph03, queries=graph03_queries))

    return graphsearch_prob
