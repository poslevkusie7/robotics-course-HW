"""Student PRM and A*: SciPy supplies only distance and neighbour primitives."""

from dataclasses import dataclass
import heapq
import math
import time

import numpy as np
from scipy.spatial import cKDTree

from .cspace import ConfigurationSpace


@dataclass(frozen=True)
class PlannerSettings:
    seed: int = 0
    samples: int = 1000
    neighbors: int = 20
    connection_radius: float = 1.5
    attempts: int = 3
    growth: float = 1.5
    planning_radius: float = 0.30

    def validate(self):
        if self.seed < 0 or self.samples < 1 or self.neighbors < 1 or self.attempts < 1:
            raise ValueError('Seed must be nonnegative; counts must be positive')
        if not math.isfinite(self.connection_radius) or self.connection_radius <= 0:
            raise ValueError('Connection radius must be positive and finite')
        if not math.isfinite(self.growth) or self.growth < 1.0:
            raise ValueError('Sample growth must be finite and at least one')
        if not math.isfinite(self.planning_radius) or self.planning_radius < 0.30:
            raise ValueError('Planning radius must be finite and at least 0.30 m')


@dataclass
class PlanningResult:
    space: ConfigurationSpace
    nodes: np.ndarray
    edges: list
    path: np.ndarray
    attempts: list
    planning_seconds: float


def astar(nodes, adjacency, start=0, goal=1):
    """Return graph vertex indices; an empty list means the goal is disconnected."""
    costs = {start: 0.0}
    parents = {}
    heuristic = lambda index: float(np.linalg.norm(nodes[index] - nodes[goal]))
    queue = [(heuristic(start), 0.0, start)]
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost > costs.get(current, math.inf):
            continue
        if current == goal:
            route = [goal]
            while route[-1] != start:
                route.append(parents[route[-1]])
            return route[::-1]
        for neighbor, length in adjacency[current]:
            candidate = cost + length
            if candidate < costs.get(neighbor, math.inf):
                costs[neighbor] = candidate
                parents[neighbor] = current
                heapq.heappush(queue, (candidate + heuristic(neighbor), candidate, neighbor))
    return []


def build_roadmap(space, nodes, neighbors, connection_radius):
    adjacency = [[] for _ in nodes]
    edges = []
    tree = cKDTree(nodes)
    count = min(neighbors + 1, len(nodes))
    distances, indices = tree.query(nodes, k=count, distance_upper_bound=connection_radius)
    candidates = set()
    for index, (nearby, lengths) in enumerate(zip(indices, distances)):
        for other, length in zip(np.atleast_1d(nearby), np.atleast_1d(lengths)):
            other = int(other)
            if other == index or other >= len(nodes) or not math.isfinite(length):
                continue
            candidates.add((min(index, other), max(index, other)))
    # Sorting removes set iteration from reproducibility and gives stable A* ties.
    for first, second in sorted(candidates):
        if not space.edge_is_free(nodes[first], nodes[second]):
            continue
        length = float(np.linalg.norm(nodes[first] - nodes[second]))
        adjacency[first].append((second, length))
        adjacency[second].append((first, length))
        edges.append((first, second))
    return edges, adjacency


def plan(raw, resolution, origin, yaw, start, goal, settings):
    settings.validate()
    began = time.perf_counter()
    space = ConfigurationSpace.build(raw, resolution, origin, yaw, settings.planning_radius)
    if not space.point_is_free(start) or not space.point_is_free(goal):
        raise ValueError('Start or goal intersects occupied configuration space')
    rng = np.random.default_rng(settings.seed)
    attempts = []
    nodes = np.asarray([start, goal], dtype=float)
    edges = []
    path = np.empty((0, 2), dtype=float)
    for attempt in range(settings.attempts):
        attempt_began = time.perf_counter()
        sample_count = math.ceil(settings.samples * settings.growth ** attempt)
        samples = space.sample_free(sample_count, rng)
        nodes = np.vstack((start, goal, samples))
        edges, adjacency = build_roadmap(space, nodes, settings.neighbors, settings.connection_radius)
        indices = astar(nodes, adjacency)
        if indices:
            path = nodes[indices]
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) else None
        attempts.append({
            'attempt': attempt + 1,
            'samples': len(samples),
            'nodes': len(nodes),
            'edges': len(edges),
            'seconds': time.perf_counter() - attempt_began,
            'path_length_m': length,
            'path_found': bool(indices),
        })
        if indices:
            break
    return PlanningResult(space, nodes, edges, path, attempts, time.perf_counter() - began)
