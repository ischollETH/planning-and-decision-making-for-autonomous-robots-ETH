from typing import Optional

import matplotlib
from dg_commons.maps.shapely_viz import ShapelyViz
from dg_commons.sim.simulator import SimContext
from matplotlib import pyplot as plt
import numpy as np
from dg_commons.sim.simulator_visualisation import ZOrders



def context_generator(seed: Optional[int] = None, noObstacles: bool=False) -> SimContext:
    from pdm4ar.exercises_def.final21.scenario import get_dgscenario
    from pdm4ar.exercises_def.final21.sim_context import _get_sim_context_static
    from shapely.strtree import STRtree

    dgscenario, goal, x0 = get_dgscenario(seed)

    if noObstacles:
        print(dgscenario.static_obstacles[0])
        dgscenario.static_obstacles = {"0": dgscenario.static_obstacles[0],}
        print(dgscenario.static_obstacles)
        print([a for a in dgscenario.static_obstacles])

        obs_shapes = [sobstacle.shape for sobstacle in dgscenario.static_obstacles.values()]
        obs_idx = [idx for idx in dgscenario.static_obstacles.keys()]
        dgscenario.strtree_obstacles = STRtree(obs_shapes, obs_idx, node_capacity=3)


    simcontext = _get_sim_context_static(dgscenario, goal, x0)
    simcontext.description = "generated-static-environment"
    return simcontext



def plot_trajectory(x, y, psi=None, time=None, label='Trajectory'):
    matplotlib.use('TkAgg')

    if time is None:
        fig, ax = plt.subplots()
        ax.plot(x, y, label=label)
        ax.set_aspect("equal")
        # plt.show(block=False)
    else:
        # print(time)
        fig, ax = plt.subplots()
        sc = ax.scatter(x, y, c=time, cmap='jet', label=label)
        ax.set_aspect("equal")
        plt.colorbar(sc, label='time', ax=ax)
        # plt.show(block=False)

    if psi is not None:
        dx = np.cos(psi)
        dy = np.sin(psi)

        width = (max(x) - min(x)) * 0.01

        ax.arrow(x[-1], y[-1], dx[-1] / 10, dy[-1] / 10, width=width)

    return fig, ax


class Plotter:
    def __init__(self, rrt_instance):
        self.outer = rrt_instance
        matplotlib.use('TkAgg')
        self.fig, self.ax = plt.subplots()
        shapely_viz = ShapelyViz(self.ax)
        plt.show(block=False)
        for s_obstacle in self.outer.static_obstacles:
            shapely_viz.add_shape(s_obstacle.shape, color=s_obstacle.geometry.color, zorder=ZOrders.ENV_OBSTACLE)
        shapely_viz.add_shape(self.outer.goal, color="orange", zorder=ZOrders.GOAL, alpha=0.5)
        plt.pause(0.1)

    def update(self, path, invalid=False):
        path_x = [st.x for st in path]
        path_y = [st.y for st in path]
        if not invalid:
            self.ax.plot(path_x, path_y,)
        else:
            self.ax.plot(path_x, path_y, '--', color='gray')
        plt.pause(0.01)
    def pause(self):
        plt.pause(1)
