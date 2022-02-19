from itertools import product
from typing import Tuple, Any

import osmnx as ox
from matplotlib import pyplot as plt
from networkx import shortest_path, NetworkXNoPath
from reprep import Report, MIME_PDF
from toolz import sliding_window
from zuper_commons.text import remove_escapes

from pdm4ar.exercises.ex03 import informed_graph_search_algo, compute_path_cost
from pdm4ar.exercises_def import Exercise, NodeColors, EdgeColors, ExIn
from pdm4ar.exercises_def.ex03.data import get_test_informed_gsproblem, InformedGraphSearchProblem


class TestValueEx3(ExIn, Tuple[InformedGraphSearchProblem, str]):
    def str_id(self) -> str:
        return str(self[1])


def exercise3_report(ex_in: TestValueEx3, ex_out=None) -> Report:
    r = Report("Exercise3-WeightedGraphSearch")

    # draw graph
    prob, algo_name = ex_in
    wG = prob.graph
    test_queries = prob.queries
    # draw graph
    figsize = None
    rfig = r.figure(cols=1)
    with rfig.plot(nid="Graph", mime=MIME_PDF, figsize=figsize) as _:
        ax = plt.gca()
        ox.plot_graph(
            wG._G, ax=ax, node_color=NodeColors.default, edge_color=EdgeColors.default, node_edgecolor="k", show=False
        )

    # run algo
    r.section(f"{algo_name}")

    for i, query in enumerate(test_queries):
        # Ground truth
        msg = f"Start: {query[0]},\tGoal: {query[1]}\n"
        search_algo = informed_graph_search_algo[algo_name]()
        rfig = r.figure(cols=2)
        try:
            gt_path = shortest_path(wG._G, query[0], query[1], weight="travel_time")
            gt_path_cost = compute_path_cost(wG, gt_path)
            path_edges = list(sliding_window(2, gt_path))
            msg += f"Ground truth path cost:\t{gt_path_cost:.2f}\n"
            ec = [EdgeColors.path if uv[:2] in path_edges else EdgeColors.default for uv in wG._G.edges]
            nc = [
                NodeColors.start if n == query[0] else (NodeColors.goal if n == query[1] else NodeColors.default)
                for n in wG._G
            ]
            with rfig.plot(nid=f"GroundTruth{i}", mime=MIME_PDF, figsize=figsize) as _:
                ax = plt.gca()
                ox.plot_graph(wG._G, ax=ax, node_color=nc, edge_color=ec, node_edgecolor="k", show=False)
        except NetworkXNoPath:
            msg += "No path exists\n"
            gt_path = None

        # Your algo
        path = search_algo.path(wG, query[0], query[1])
        if path:
            path_cost = compute_path_cost(wG, path)
            msg += f"Your path cost:\t{path_cost:.2f}\n"
            path_edges = list(sliding_window(2, path))
            ec = [EdgeColors.path if uv[:2] in path_edges else EdgeColors.default for uv in wG._G.edges]
            nc = [
                NodeColors.start if n == query[0] else (NodeColors.goal if n == query[1] else NodeColors.default)
                for n in wG._G
            ]
            with rfig.plot(nid=f"YourPath{i}", mime=MIME_PDF, figsize=figsize) as _:
                ax = plt.gca()
                ox.plot_graph(wG._G, ax=ax, node_color=nc, edge_color=ec, node_edgecolor="k", show=False)
        else:
            msg += "Your algo did not find any path.\n"

        if path == gt_path and gt_path:
            msg += "Your path matches the shortest path\n"

        r.text(f"{algo_name}-query{i}", text=remove_escapes(msg))

    return r


def algo_placeholder(ex_in):
    return None


def get_exercise3() -> Exercise:
    test_wgraphs = get_test_informed_gsproblem()
    test_values = list()
    for ab in product(test_wgraphs, informed_graph_search_algo):
        test_values.append(TestValueEx3(ab))

    return Exercise[TestValueEx3, Any](
        desc="This exercise is about graph search",
        algorithm=algo_placeholder,
        report=exercise3_report,
        test_values=test_values,
    )
