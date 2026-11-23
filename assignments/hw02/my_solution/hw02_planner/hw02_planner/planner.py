"""The whole planning pipeline, with no ROS in it (so it is easy to test).

    raw map -> configuration space -> PRM -> A*  (retry with more samples)

Retry policy: try each sample count in SAMPLE_SCHEDULE in turn, each time with a
fresh roadmap from a random generator seeded with (seed, attempt number). Stop at
the first attempt that finds a path. If every attempt fails we report "no path".
The budget is finite, so a failure only means "not found with these samples",
not "impossible".
"""
import time
from dataclasses import dataclass, field

import numpy as np

from .astar import astar, path_length
from .cspace import build_configuration_space
from .prm import GOAL, START, build_roadmap

SAMPLE_SCHEDULE = (1000, 2000, 5000)
NEIGHBOURS_PER_NODE = 12
MAX_EDGE_LENGTH = 1.5   # metres


@dataclass
class Attempt:
    samples_drawn: int
    nodes: int
    edges: int
    seconds: float
    found_path: bool


@dataclass
class PlanResult:
    cspace: object
    roadmap: object = None              # roadmap of the last attempt
    waypoints: list = None              # [(x, y), ...] start ... goal, or None
    attempts: list = field(default_factory=list)
    problem: str = ''                   # explanation when there is no path

    @property
    def found(self):
        return self.waypoints is not None

    @property
    def length(self):
        return path_length(self.waypoints) if self.found else float('nan')

    @property
    def total_seconds(self):
        return sum(a.seconds for a in self.attempts)


def plan(raw_map, start, goal, seed=0, sample_schedule=SAMPLE_SCHEDULE,
         neighbours_per_node=NEIGHBOURS_PER_NODE, max_edge_length=MAX_EDGE_LENGTH):
    cspace = build_configuration_space(raw_map)
    result = PlanResult(cspace=cspace)

    for label, point in (('start', start), ('goal', goal)):
        if not cspace.is_free(point[0], point[1]):
            result.problem = f'{label} {tuple(point)} is inside a configuration-space obstacle'
            return result

    for attempt_number, sample_count in enumerate(sample_schedule):
        began = time.perf_counter()
        rng = np.random.default_rng([seed, attempt_number])   # reproducible per seed and attempt
        roadmap = build_roadmap(cspace, start, goal, sample_count, rng,
                                neighbours_per_node, max_edge_length)
        node_path = astar(roadmap, START, GOAL)
        result.roadmap = roadmap
        result.attempts.append(Attempt(samples_drawn=sample_count,
                                       nodes=len(roadmap.nodes),
                                       edges=len(roadmap.edges()),
                                       seconds=time.perf_counter() - began,
                                       found_path=node_path is not None))
        if node_path is not None:
            result.waypoints = [tuple(map(float, roadmap.nodes[n])) for n in node_path]
            return result

    result.problem = (f'no roadmap path after {len(sample_schedule)} attempts '
                      f'({", ".join(map(str, sample_schedule))} samples)')
    return result
