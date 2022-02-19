from typing import Tuple, Any

from reprep import Report, MIME_PDF
from zuper_commons.text import remove_escapes

from pdm4ar.exercises.ex02 import graph_search_algo
from pdm4ar.exercises_def import Exercise, ExIn
from pdm4ar.exercises_def.ex02.data import *


@dataclass
class NodeColors:
    default: str = "cyan"
    start: str = "orange"
    goal: str = "green"


@dataclass
class EdgeColors:
    default: str = "dimgray"
    path: str = "red"
    gt_path: str = "green"


class TestValueEx2(ExIn, Tuple[GraphSearchProblem, str]):
    def str_id(self) -> str:
        return str(self[1])


def exercise2_report(ex_in, ex_out=None) -> Report:
    r = Report("Exercise2-GraphSearch")
    # draw graph
    graph_search_prob, algo_name = ex_in
    test_graph = graph_search_prob.graph
    test_queries = graph_search_prob.queries
    G = nx.DiGraph()
    G.add_nodes_from(test_graph.keys())
    pic_size = max(10, int(G.number_of_nodes() / 10))
    figsize = (pic_size, pic_size)
    for n, successors in test_graph.items():
        G.add_edges_from(
            product(
                [
                    n,
                ],
                successors,
            )
        )
    # draw graph
    pos = nx.get_node_attributes(G, "pos")
    if not pos:
        pos = nx.spring_layout(G)
    rfig = r.figure(cols=1)
    with rfig.plot(nid="Graph", mime=MIME_PDF, figsize=figsize) as _:
        nx.draw(G, pos=pos, with_labels=True, node_color=NodeColors.default)

    # run algo
    r.section(f"{algo_name}")

    for i, query in enumerate(test_queries):
        # Set all edge color attribute to black
        for e in G.edges():
            G[e[0]][e[1]]["color"] = EdgeColors.default
        msg = f"Start: {query[0]},\tGoal: {query[1]}\n"

        search_algo = graph_search_algo[algo_name]()
        path = search_algo.search(test_graph, query[0], query[1])

        if path:
            path_str = ""
            for node in path:
                path_str += f"{node}->"
            msg += "Found path:" + path_str[:-2]

            path_edges: List[Tuple] = []
            for j in range(len(path) - 1):
                path_edges.append((path[j], path[j + 1]))

            rfig = r.figure(cols=1)
            with rfig.plot(nid=f"Path{i}", mime=MIME_PDF, figsize=figsize) as _:
                node_colors = [
                    NodeColors.start if n == query[0] else (NodeColors.goal if n == query[1] else NodeColors.default)
                    for n in G
                ]
                nx.draw_networkx_nodes(G, pos=pos, node_color=node_colors)
                nx.draw_networkx_edges(G, pos=pos, edge_color=EdgeColors.default)
                nx.draw_networkx_edges(G, pos=pos, edgelist=path_edges, edge_color=EdgeColors.path)
                nx.draw_networkx_labels(G, pos=pos)
        else:
            msg += "No path found"
        r.text(f"{algo_name}-query{i}", text=remove_escapes(msg))

    return r


def algo_placeholder(ex_in):
    return None


def get_exercise2() -> Exercise:
    graph_search_problems = get_graph_search_problems()
    graph_search_algos = graph_search_algo.keys()

    test_values = list()
    for ab in product(graph_search_problems, graph_search_algos):
        test_values.append(TestValueEx2(ab))

    return Exercise[TestValueEx2, Any](
        desc="This exercise is about graph search",
        algorithm=algo_placeholder,
        report=exercise2_report,
        test_values=test_values,
    )
