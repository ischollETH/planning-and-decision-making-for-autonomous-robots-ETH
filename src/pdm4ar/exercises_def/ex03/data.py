from dataclasses import dataclass
from typing import List, Set

import osmnx as ox
from frozendict import frozendict
from networkx import MultiDiGraph

from pdm4ar.exercises.ex02.structures import Query
from pdm4ar.exercises.ex03.structures import WeightedGraph
from pdm4ar.exercises_def import networkx_2_adjacencylist, queries_from_adjacency

_fast = (
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "motorway_link",
    "trunk_link",
    "primary_link",
    "secondary_link",
    "escape",
    "track",
)
_slow = ("tertiary", "residential", "tertiary_link", "living_street")
_other = ("unclassified", "road", "service")


@dataclass
class InformedGraphSearchProblem:
    graph: WeightedGraph
    queries: Set[Query]


def _find_speed(row) -> float:
    if row["highway"] in _fast:
        return 100 / 3.6
    elif row["highway"] in _slow:
        return 50 / 3.6
    elif row["highway"] in _other:
        return 70 / 3.6
    else:
        return 5 / 3.6


def add_travel_time_weight(G: MultiDiGraph) -> MultiDiGraph:
    nodes, edges = ox.graph_to_gdfs(G)
    edges = edges.assign(speed=edges.apply(_find_speed, axis=1))
    edges["travel_time"] = edges["length"] / edges["speed"]
    UG = ox.graph_from_gdfs(nodes, edges)
    return UG


def networkx_2_weighted_graph(G: MultiDiGraph) -> WeightedGraph:
    G = add_travel_time_weight(G)
    adj = networkx_2_adjacencylist(G)
    weights = dict()
    for source, successors in adj.items():
        for dest in successors:
            min_weight = min([edge["travel_time"] for edge in G[source][dest].values()])
            assert isinstance(min_weight, float)
            assert min_weight > 0
            weights[(source, dest)] = min_weight
    wG = WeightedGraph(adj_list=adj, weights=frozendict(weights), _G=G)
    return wG


def get_test_informed_gsproblem() -> List[InformedGraphSearchProblem]:
    G_empire = ox.graph_from_address("350 5th Ave, New York, New York", network_type="drive")
    G_eth = ox.graph_from_address("Rämistrasse 101, 8092 Zürich, Switzerland", network_type="drive")
    G_milan = ox.graph_from_address("P.za del Duomo, 20122 Milano MI, Italy", network_type="drive")
    test_graphs = map(add_travel_time_weight, [G_empire, G_eth, G_milan])
    test_wgraphs = map(networkx_2_weighted_graph, test_graphs)

    data_in: List[InformedGraphSearchProblem] = []
    for G in test_wgraphs:
        q = queries_from_adjacency(G.adj_list, n=5)
        p = InformedGraphSearchProblem(
            graph=G,
            queries=q,
        )
        data_in.append(p)

    return data_in


# if __name__ == '__main__':
#
#     for test_p in get_test_informed_gsproblem():
#         ox.plot_graph(test_p.graph._G)
