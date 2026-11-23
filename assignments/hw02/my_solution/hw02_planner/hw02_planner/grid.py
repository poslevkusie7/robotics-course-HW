"""A small wrapper around an occupancy grid.

Conventions (same as nav_msgs/OccupancyGrid):
  * the array is indexed  occupied[row, column]
  * column grows with world X, row grows with world Y
  * cell (row=0, column=0) has its lower-left corner at the map origin
The map origin is assumed to have no rotation (true for this assignment).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class GridMap:
    occupied: np.ndarray   # bool array, shape (rows, columns); True = robot may not be here
    resolution: float      # metres per cell
    origin_x: float        # world X of the grid's lower-left corner
    origin_y: float        # world Y of the grid's lower-left corner

    @classmethod
    def from_msg(cls, msg):
        """Build from a nav_msgs/OccupancyGrid. Unknown cells (-1) count as occupied."""
        info = msg.info
        values = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return cls(occupied=(values != 0),
                   resolution=info.resolution,
                   origin_x=info.origin.position.x,
                   origin_y=info.origin.position.y)

    @property
    def rows(self):
        return self.occupied.shape[0]

    @property
    def columns(self):
        return self.occupied.shape[1]

    @property
    def x_range(self):
        return self.origin_x, self.origin_x + self.columns * self.resolution

    @property
    def y_range(self):
        return self.origin_y, self.origin_y + self.rows * self.resolution

    def world_to_cell(self, x, y):
        """World coordinates (metres) -> (row, column) of the containing cell."""
        column = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(int)
        row = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(int)
        return row, column

    def cell_center(self, row, column):
        """(row, column) -> world (x, y) of the cell centre."""
        x = self.origin_x + (np.asarray(column) + 0.5) * self.resolution
        y = self.origin_y + (np.asarray(row) + 0.5) * self.resolution
        return x, y

    def is_free(self, x, y):
        """True where the point lies inside the map AND in a free cell.

        Works on single numbers or on whole numpy arrays of points.
        Points outside the map are reported as not free.
        """
        row, column = self.world_to_cell(x, y)
        inside = (row >= 0) & (row < self.rows) & (column >= 0) & (column < self.columns)
        free = np.zeros(np.shape(row), dtype=bool)
        free[inside] = ~self.occupied[row[inside], column[inside]]
        return free

    def to_msg_data(self):
        """Flatten back to OccupancyGrid data: 100 = occupied, 0 = free, row-major."""
        return np.where(self.occupied, 100, 0).astype(np.int8).ravel().tolist()
