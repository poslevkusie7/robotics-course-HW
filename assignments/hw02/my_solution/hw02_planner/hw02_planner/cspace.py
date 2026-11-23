"""Configuration space for a round robot.

The robot is a disc and turning does not change its shape, so a configuration is
just its centre (x, y). The centre is allowed in a cell only if the whole disc of
radius `robot_radius` around it stays clear of every obstacle cell.

How it is computed
------------------
For every cell, `distance_transform_edt` gives the distance from that cell's
centre to the centre of the nearest obstacle cell. An obstacle cell is a square,
not a point: its nearest corner can be up to half a diagonal (res / sqrt 2)
closer than its centre. So a cell is blocked when

    distance_between_centres <= robot_radius + res / sqrt(2)

The map border is already a ring of occupied cells, so walls are inflated by the
same rule, and unknown cells were already turned into obstacles by GridMap.
"""
import math

from scipy import ndimage

from .grid import GridMap

PLANNING_RADIUS = 0.25 + 0.05   # robot footprint + safety margin (metres)


def build_configuration_space(raw: GridMap, robot_radius=PLANNING_RADIUS) -> GridMap:
    distance_in_cells = ndimage.distance_transform_edt(~raw.occupied)
    distance_m = distance_in_cells * raw.resolution
    blocked_within = robot_radius + raw.resolution / math.sqrt(2)
    return GridMap(occupied=(distance_m <= blocked_within),
                   resolution=raw.resolution,
                   origin_x=raw.origin_x,
                   origin_y=raw.origin_y)
