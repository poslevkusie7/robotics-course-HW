"""Offline tests for grid conversion, inflation, edge checking and A*.

Run from the repository root (inside the pixi shell, environment sourced):
    python -m pytest -q assignments/hw02/my_solution/hw02_planner/test
"""
import math

import numpy as np
import pytest

from hw02_planner.astar import astar
from hw02_planner.cspace import build_configuration_space
from hw02_planner.grid import GridMap
from hw02_planner.planner import plan
from hw02_planner.prm import Roadmap, edge_is_free

RES = 0.025


def empty_room(width_m=2.0, height_m=2.0, border=True):
    occupied = np.zeros((round(height_m / RES), round(width_m / RES)), dtype=bool)
    if border:
        occupied[[0, -1], :] = True
        occupied[:, [0, -1]] = True
    return GridMap(occupied=occupied, resolution=RES, origin_x=0.0, origin_y=0.0)


# ---- grid conversion -------------------------------------------------------

def test_world_to_cell_and_back():
    grid = GridMap(np.zeros((240, 320), bool), RES, 0.0, 0.0)
    row, column = grid.world_to_cell(1.0, 3.0)
    assert (row, column) == (120, 40)              # row is Y, column is X
    x, y = grid.cell_center(row, column)
    assert abs(x - 1.0) <= RES and abs(y - 3.0) <= RES


def test_points_outside_the_map_are_not_free():
    grid = empty_room(border=False)
    assert not grid.is_free(-0.1, 1.0)
    assert not grid.is_free(1.0, 2.1)
    assert grid.is_free(1.0, 1.0)


# ---- configuration space ---------------------------------------------------

def test_single_obstacle_cell_grows_into_disc_of_planning_radius():
    grid = empty_room(border=False)
    grid.occupied[40, 40] = True                     # one cell around (1.0125, 1.0125)
    cspace = build_configuration_space(grid, robot_radius=0.30)
    cx, cy = grid.cell_center(40, 40)
    assert not cspace.is_free(cx + 0.29, cy)         # inside radius: blocked
    assert not cspace.is_free(cx, cy - 0.30)         # on the radius: blocked
    assert cspace.is_free(cx + 0.36, cy)             # clearly outside: free


def test_unknown_cells_are_treated_as_obstacles():
    class Info:                                       # minimal stand-in for an OccupancyGrid
        width, height, resolution = 80, 80, RES
        class origin:
            class position:
                x = y = 0.0
    class Msg:
        info = Info
        data = [0] * (80 * 80)
    Msg.data[40 * 80 + 40] = -1
    grid = GridMap.from_msg(Msg)
    assert grid.occupied[40, 40]


def test_walls_are_inflated():
    cspace = build_configuration_space(empty_room(), robot_radius=0.30)
    assert not cspace.is_free(0.25, 1.0)             # too close to the left wall
    assert cspace.is_free(0.40, 1.0)


# ---- edge checking ---------------------------------------------------------

def test_edge_through_obstacle_is_rejected_even_if_both_ends_are_free():
    grid = empty_room(border=False)
    grid.occupied[:, 40] = True                      # thin vertical wall at x = 1.0
    cspace = build_configuration_space(grid, robot_radius=0.30)
    a, b = (0.4, 1.0), (1.6, 1.0)
    assert cspace.is_free(*a) and cspace.is_free(*b)
    assert not edge_is_free(cspace, a, b)


def test_edge_that_leaves_the_map_is_rejected():
    cspace = empty_room(border=False)
    assert not edge_is_free(cspace, (0.5, 0.5), (2.5, 0.5))


def test_edge_in_open_space_is_accepted():
    cspace = build_configuration_space(empty_room(), robot_radius=0.30)
    assert edge_is_free(cspace, (0.5, 0.5), (1.5, 1.5))


# ---- A* --------------------------------------------------------------------

def make_graph(points, edges):
    roadmap = Roadmap(nodes=np.array(points, dtype=float))
    for a, b in edges:
        roadmap.add_edge(a, b)
    return roadmap


def test_astar_picks_the_shorter_route():
    #      2 (long detour, high above)
    #  0 ---- 3 ---- 1
    points = [(0, 0), (2, 0), (1, 5), (1, 0)]
    roadmap = make_graph(points, [(0, 2), (2, 1), (0, 3), (3, 1)])
    assert astar(roadmap, 0, 1) == [0, 3, 1]


def test_astar_returns_none_when_goal_is_disconnected():
    roadmap = make_graph([(0, 0), (5, 0), (1, 0)], [(0, 2)])
    assert astar(roadmap, 0, 1) is None


# ---- whole pipeline on the real rooms -------------------------------------

def supplied_room(layout):
    scenario = pytest.importorskip('hw02_sim.scenario')
    return GridMap(scenario.raw_map(layout) != 0, RES, 0.0, 0.0)


@pytest.mark.parametrize('layout', ['wide', 'mixed'])
def test_solvable_rooms_get_a_collision_free_path(layout):
    result = plan(supplied_room(layout), (1.0, 3.0), (7.0, 3.0), seed=0)
    assert result.found
    for a, b in zip(result.waypoints, result.waypoints[1:]):
        assert edge_is_free(result.cspace, a, b)


def test_blocked_room_reports_no_path():
    result = plan(supplied_room('blocked'), (1.0, 3.0), (7.0, 3.0), seed=0)
    assert not result.found and result.waypoints is None


def test_same_seed_gives_same_path():
    room = supplied_room('wide')
    first = plan(room, (1.0, 3.0), (7.0, 3.0), seed=1).waypoints
    second = plan(room, (1.0, 3.0), (7.0, 3.0), seed=1).waypoints
    assert first == second
