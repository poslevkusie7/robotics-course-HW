"""A* search on the roadmap graph.

cost so far  g(n): sum of Euclidean edge lengths from the start
heuristic    h(n): straight-line distance from n to the goal

h never overestimates the remaining distance (no path is shorter than a straight
line), so the first time the goal is taken off the queue its path is the
shortest path that exists in the roadmap.
"""
import heapq
import itertools

import numpy as np


def astar(roadmap, start, goal):
    """Return the list of node indices from start to goal, or None if unreachable."""
    nodes = roadmap.nodes

    def heuristic(n):
        return float(np.linalg.norm(nodes[n] - nodes[goal]))

    tie_breaker = itertools.count()       # keeps heap entries comparable when f is equal
    queue = [(heuristic(start), next(tie_breaker), start)]
    cost_so_far = {start: 0.0}
    came_from = {start: None}
    finished = set()

    while queue:
        _, _, current = heapq.heappop(queue)
        if current == goal:
            return _rebuild_path(came_from, goal)
        if current in finished:            # an older, worse copy of this node
            continue
        finished.add(current)

        for neighbour, edge_length in roadmap.neighbours.get(current, []):
            new_cost = cost_so_far[current] + edge_length
            if new_cost < cost_so_far.get(neighbour, float('inf')):
                cost_so_far[neighbour] = new_cost
                came_from[neighbour] = current
                heapq.heappush(queue, (new_cost + heuristic(neighbour), next(tie_breaker), neighbour))
    return None


def _rebuild_path(came_from, goal):
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    return path[::-1]


def path_length(points):
    points = np.asarray(points)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
