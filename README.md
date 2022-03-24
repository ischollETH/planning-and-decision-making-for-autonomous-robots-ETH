# planning-and-decision-making-for-autonomous-robots-ETH

This repository contains the python programming exercises and final project for the course 'Planning and Decision Making for Autonomous Robots' (Prof. Frazzoli) @ ETH Zurich.
For detailed descriptions, see the documentation in the `docs/` folder.

The final project was awarded the full grade bonus.

Credit for skeleton codes, exercise descriptions and setups go to Prof. Frazzoli and his teaching assistants ([course exercise website](https://idsc-frazzoli.github.io/PDM4AR-exercises/)).

## Exercise 1: Lexicographic comparison

This introductory exercise implements a function that compares two vectors according to a lexicographic order. For each entry, lower is better. Such an order can be interesting when ranking and comparing constraints and constraint violations respectively plans for an autonomous vehicle.

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/ex01_LexicographicComparison.png width="600" title="Results of exercise 1 on lexicographic comparison">
</p>


## Exercise 2: Graph search

This exercise implements the algorithms for breadth first search (BFS), depth first search (DFS) and Iterative Deepening for finding paths between a starting and terminal node on a simple graph without weights on the edges. A sample result for Iterative Deepening can be seen in the following:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/ex02_IterativeDeepening.png width="600" title="Results of exercise 2 using Iterative Deepening">
</p>


## Exercise 3:  Informed graph search

This exercise implements the algorithms for uniform cost search (UCS), greedy best first search (DFS) and A* for finding paths between a starting and terminal node on a graph with weighed edges. A sample result using the A* algorithm can be seen in the following:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/ex03_AstarAlgorithm.png width="1200" title="Results of exercise 3 using the A* algorithm">
</p>


## Exercise 4: Dynamic Programming

This exercise implements Value Iteration (VI) and Policy Iteration (PI) for a particular stationary Markov Decision Process (MDP). An autonomous robot is parachuted in a remote area of the planet for a rescue mission. An optimal policy to reach the GOAL cell (reward +10; visualized in red) has to be computed. Some cells in the map are simply GRASS (reward -1; green), some others are of type SWAMP (reward -5; light blue).
When in a specific cell, the robot can move SOUTH, NORTH, EAST, WEST (if not next to a boundary) and if arrived at the GOAL, it can STAY. Applying an action from any given cell to get to an adjacent cell is successful with a probability of 1.
The result for a given setup using e.g. Value Iteration can be seen in the following:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/ex04_ValueIteration.png width="1200" title="Results of exercise 4 using Value Iteration">
</p>


## Exercise 5: Geometry and poses

This exercise was implemented as a [jupyter notebook](https://jupyter.org/) (see folder `notebooks/`), it implements different geometric transforms and pose transformations. For example, an interpolation on poses like in the following can be calculated and visualized using the correct transforms:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/ex05_PoseInterpolation.png width="400" title="Results of exercise 5 for a Pose Interpolation">
</p>


## Final Project - Spacecraft path planning

For this group project the proposed situation is a s follows: a spacecraft has been launched into deep space with the mission of bringing supplies to the interplanetary space station. Unexpectedly a field of asteroids is surrounding the interplanetary station at the moment of arrival. The task is to implement a path planning agent able to reach the docking area in the best possible way.

The simulation terminates upon one of the following cases:
- The agent reaches the goal (the spacecraft center of gravity (CoG) is inside the goal area)
- The agent crashes into an obstacle
- The maximum simulation time is reached

The spacecraft has a left and right thruster at the back that can be activated to push the spacecraft forward or backward. Applying differential thrust will also cause the spacecraft to rotate. An illustrative figure of the spacecraft model is shown below.

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/final21_spacecraftModel.png width="500" title="Spacecraft model">
</p>

The agent can deal with different instances of the world by changing the parameters that create the space/obstacles/goal region and different initial conditions.
The solution was benchmarked against two scenarios. One containing only static obstacles (purple asteroids), one containing also dynamic obstacles (green asteroids). The initial scene looks as follows:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/final21_initialSituation.jpeg width="500" title="Initial scene with the spacecraft, goal area in yellow, static asteroids in purple and dynamic asteroids in green">
</p>

It was decided in the group to implement a rapidly-exploring random trees (RRT respectively RRT*) planning algorithm. To sample new points, a simple halton sequence was used, for the nearest neighbor search kd-trees were implemented. The following are some impressions of intermediate results during the execution of the RRT algorithm:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/final21_rrt2.jpeg width="400" title="possible continuations of paths">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/final21_rrt1.jpeg width="400" title="first complete path found after few iterations">
</p>

Finally, an optimization based steering function was coded. In a static environment, the spacecraft can reach the goal as seen in the following:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/final21_StaticEnvironment.gif width="750" title="Spacecraft navigating through a static scene">
</p>

As the RRT algorithm does not necessarily return ideal shortest paths, the movement of the spacecraft also is not ideal, but still reaches the goal in rather short time and without taking too big detours. Similarly, one can observe the spacecraft navigating through a scene including dynamic asteroids; here the movement of the dynamic obstacles was calculated and predicted for future timesteps, and the planning updated whenever the dynamic obstacles deviated from the expected line of movement:

<p align="center">
  <img src=https://github.com/ischollETH/planning-and-decision-making-for-autonomous-robots-ETH/blob/master/images/final21_DynamicEnvironment.gif width="750" title="Spacecraft navigating through a dynamic scene">
</p>

Again, some of the movements of the spacecraft are a bit counter-intuitive and not ideal. With more time on the project, such flaws could have been further improved; but the spacecraft still reaches the goal on a more or less direct route, successfully avoiding dynamic obstacles too.
