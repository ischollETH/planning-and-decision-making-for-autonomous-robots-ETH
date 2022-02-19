# Graph search :computer:

<table>
  <tr>
    <th><i>Prerequisites:</i></th><td><a href="./00-preliminaries.html" target="_top">Preliminaries</a></td><td><a href="./01-hello-world.html" target="_top">Hello-world</a></td>
  </tr>
</table>

## Breadth/Depth first search

As a first exercise we are going to implement Breadth/Depth first search and Iterative Deepening.

#### Graph structures

In this exercise we represent a directed graph via an *adjacency list*. Note that this is not the only possible
representation (e.g. adjacency matrices,...) but it is a very convenient one for graph search problems if the graph is
known a priori.

Given a generic type `X` for the nodes we associate to each node the list of successors, thus:

```python
AdjacencyList = Mapping[X, Set[X]]
```

#### Task

The task is to implement the _abstractmethod_ `search` for the different search techniques (`exercises/ex02/algo.py`).
Given a graph, a starting node, and a goal node the task is to return a sequence of states (_transitions_) from start to
goal.

```python
@abstractmethod
def search(self, graph: AdjacencyList, start: X, goal: X) -> List[X]:
    """
    :param graph: The given graph as an adjacency list
    :param start: The initial state (i.e. a node)
    :param goal: The goal state (i.e. a node)
    :return: The path from start to goal as a Sequence of states, None if a path does not exist
    """
    pass
```

These algorithms are going to be tested on a few different graphs, each containing randomly generated queries (start &
goal node). In the exercise report you will find one sub-report for each combination of algorithm and graph.

##### Update your repo

Update your repo using

```shell
make update
```

this will pull the new exercises in your forked repo. If you get some merge conflicts it is because you might have
modified/moved files that you were not supposed to touch (i.e., outside the `exercises` folder).

###### Run the exercise

```shell
make run-exercise2
```

If the graph is large, you can check your results by inspecting the pdf of the image.
Click on *data nodes* button at the top of the report and download the pdf.

#### Food for thoughts

* Which of the graph search algorithms you implemented are better suited for different topologies of graphs?
* Does the presence of cycles in the graph affects any of the algorithms? If yes, why? Which modifications would you do to improve?
* Are the paths that you found the _shortest_ path?
