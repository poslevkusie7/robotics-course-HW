"""Probabilistic Roadmap (PRM).

1. Sample random points; keep the ones that are free in configuration space.
2. Join each point to its nearest neighbours with straight edges, but only if
   the WHOLE edge stays in free configuration space (see `edge_is_free`).

Node 0 is always the start and node 1 is always the goal, so they are
connected by exactly the same rules as the random samples.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from .grid import GridMap

START, GOAL = 0, 1


@dataclass
class Roadmap:
    nodes: np.ndarray                                 # shape (N, 2): x, y of every node
    neighbours: dict = field(default_factory=dict)    # node -> list of (other node, edge length)

    def add_edge(self, a, b):
        length = float(np.linalg.norm(self.nodes[a] - self.nodes[b]))
        self.neighbours.setdefault(a, []).append((b, length))
        self.neighbours.setdefault(b, []).append((a, length))

    def edges(self):
        """Every edge once, as (a, b) with a < b."""
        return [(a, b) for a, linked in self.neighbours.items() for b, _ in linked if a < b]


def edge_is_free(cspace: GridMap, a, b):
    """Check the straight segment a -> b along its entire length.

    Points are taken every half cell, so no cell the segment crosses can be
    skipped. The edge is rejected if any point is in an occupied configuration-
    space cell or outside the map.
    """
    length = float(np.hypot(b[0] - a[0], b[1] - a[1]))
    step = cspace.resolution / 2
    count = max(2, int(np.ceil(length / step)) + 1)
    t = np.linspace(0.0, 1.0, count)
    xs = a[0] + t * (b[0] - a[0])
    ys = a[1] + t * (b[1] - a[1])
    return bool(cspace.is_free(xs, ys).all())


def sample_free_points(cspace: GridMap, count, rng):
    """Draw `count` uniform random points in the map and keep only the free ones."""
    (x_min, x_max), (y_min, y_max) = cspace.x_range, cspace.y_range
    points = rng.uniform([x_min, y_min], [x_max, y_max], size=(count, 2))
    return points[cspace.is_free(points[:, 0], points[:, 1])]


def build_roadmap(cspace: GridMap, start, goal, sample_count, rng,
                  neighbours_per_node=12, max_edge_length=1.5):
    """Sample, then connect every node to up to k nearest neighbours within max_edge_length."""
    samples = sample_free_points(cspace, sample_count, rng)
    nodes = np.vstack([start, goal, samples])
    roadmap = Roadmap(nodes=nodes)

    tree = cKDTree(nodes)
    # k + 1 because the closest "neighbour" of a node is the node itself.
    distances, indices = tree.query(nodes, k=neighbours_per_node + 1,
                                    distance_upper_bound=max_edge_length)
    already_checked = set()
    for a in range(len(nodes)):
        for distance, b in zip(distances[a], indices[a]):
            if b == a or not np.isfinite(distance):   # itself, or no neighbour in range
                continue
            pair = (min(a, b), max(a, b))
            if pair in already_checked:
                continue
            already_checked.add(pair)
            if edge_is_free(cspace, nodes[a], nodes[b]):
                roadmap.add_edge(a, b)
    return roadmap
