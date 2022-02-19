import numpy as np
from sklearn.neighbors import KDTree


def in_reachable_region(state, node, dist):
    """ computes the region that can be reasonably reached with the current heading angle and velocities """
    pos = state[:2]
    dxdy = node - pos
    node_angle = np.arctan2(dxdy[1], dxdy[0])
    if node_angle < 0:
        node_angle = 2*np.pi + node_angle
    angle = abs(state[2]-node_angle)
    if angle > np.pi:
        angle = 2*np.pi - angle
    a = 0.1
    b = 2
    target = a*(dist**b)
    if angle <= target:
        return True
    else:
        return False


class Tree:
    def __init__(self, nodes):
        self.nodes = nodes[:, :2]
        self.tree = KDTree(nodes[:, :2], leaf_size=2)

    def find_k_nearest_neighbors(self, state, k):
        """ assume state is of form [x, y, phi, x_dot, y_dot, phi_dot] """
        pos = state[:2]
        dists, idxs = self.tree.query([pos], k=k)  # indices and dists of k closest neighbors
        return idxs, dists

    def find_reachable_neighbor(self, state, k):
        """ assume state is of form [x, y, phi, x_dot, y_dot, phi_dot] """
        pos = state[:2]
        pos = np.expand_dims(pos, axis=0)
        dists, idxs = self.tree.query(pos, k=k)  # indices and dists of k closest neighbors
        dists, idxs = np.squeeze(dists, axis=0), np.squeeze(idxs, axis=0)
        for i, dist in enumerate(dists):
            idx = idxs[i]
            if in_reachable_region(state, self.nodes[idx], dist):
                neighbor_idx = idx
                return neighbor_idx, dist

        print('no ideal nearest neighbor within desirable region found!!')
        return 0, dist
