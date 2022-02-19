from dataclasses import dataclass
from typing import Tuple, Mapping, Optional, Any

from networkx import MultiDiGraph

from pdm4ar.exercises.ex02.structures import AdjacencyList, X


class EdgeNotFound(Exception):
    pass


class NodePropertyNotFound(Exception):
    pass


@dataclass
class WeightedGraph:
    adj_list: AdjacencyList
    weights: Mapping[Tuple[X, X], float]
    _G: MultiDiGraph

    def get_weight(self, u: X, v: X) -> Optional[float]:
        """
        :param u: The "from" of the edge
        :param v: The "to" of the edge
        :return: The weight associated to the edge, raises an Exception if the edge does not exist
        """
        try:
            return self.weights[(u, v)]
        except KeyError:
            raise EdgeNotFound(f"Cannot find weight for edge: {(u, v)}")

    def get_node_attribute(self, node_id: X, attribute: str) -> Any:
        """
        :param node_id: The node id
        :param attribute: The node attribute name
        :return: The corresponding value
        """
        return self._G.nodes[node_id][attribute]
