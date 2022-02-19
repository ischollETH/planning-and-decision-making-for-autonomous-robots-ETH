# Informed Graph Search :computer:

<table>
  <tr>
    <th><i>Prerequisites:</i></th><td><a href="./00-preliminaries.html" target="_top">Preliminaries</a></td><td><a href="./01-hello-world.html" target="_top">Hello-world</a></td>
  </tr>
</table>

## Informed graph search

### Graph structures

In this exercise we need to augment the `AdjacencyList` seen in <a href="./02-graphsearch.html" target="_top">Exercise
2</a>
to keep track of the weights of the edges. A simple extension is the following:

```python
@dataclass
class WeightedGraph:
    adj_list: AdjacencyList
    weights: Mapping[Tuple[X, X], float]

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
```

To properly implement a heuristic, we will need to get some property from the nodes (e.g., their position on the map).
For that you can use the method `get_node_attribute`. For example, in this exercises the graphs will be maps; you can
retrieve the _longitude_ and _latitude_ of a node as follows:

```python
wG: WeightedGraph
node: X  # the node id
lon = wG.get_node_attribute(node, "x")
lat = wG.get_node_attribute(node, "y")
```

### Task

Implement the following algorithms (`src/pdm4ar/exercises/ex03/algo.py`):

```python
class UniformCostSearch(InformedGraphSearch):
    def path(self, graph: WeightedGraph, start: X, goal: X) -> Optional[List[X]]:
        # todo
        pass


class GreedyBestFirst(InformedGraphSearch):
    def path(self, graph: WeightedGraph, start: X, goal: X) -> Optional[List[X]]:
        # todo
        pass


class Astar(InformedGraphSearch):
    def path(self, graph: WeightedGraph, start: X, goal: X) -> Optional[List[X]]:
        # todo
        pass
```

Note that the weights for the graph correspond to the expected travel time.

#### Update your repo

Update your repo using

```shell
make update
```

this will pull the new exercises in your forked repo. If you get some merge conflicts it is because you might have
modified/moved files that you were not supposed to touch (i.e., outside the `exercises` folder).

###### Run the exercise

```shell
make run-exercise3
```

#### Food for thoughts

* Which of the methods above is supposed to always find the shortest path?
* What are valid heuristic you can think of for the A* algorithm? Given the different topology of the three cities, do you expect some to work better on specific cities?
