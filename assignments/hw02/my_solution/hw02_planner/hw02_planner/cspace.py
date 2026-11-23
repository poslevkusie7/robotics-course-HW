"""Conservative configuration-space construction and closed-cell edge checks."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class ConfigurationSpace:
    raw: np.ndarray
    blocked: np.ndarray
    resolution: float
    origin: tuple
    yaw: float
    radius: float

    @classmethod
    def build(cls, raw, resolution, origin, yaw=0.0, radius=0.30):
        raw = np.asarray(raw, dtype=np.int16)
        if raw.ndim != 2 or min(raw.shape) < 1:
            raise ValueError('Map must be a nonempty 2D array')
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError('Map resolution must be positive and finite')
        if not all(math.isfinite(value) for value in (*origin, yaw, radius)):
            raise ValueError('Map geometry and planning radius must be finite')
        if radius < 0.30:
            raise ValueError('HW02 requires a planning radius of at least 0.30 m')

        # Only explicit zero is free. Padding also treats the map exterior as occupied.
        free = np.pad(raw == 0, 1, constant_values=False)
        distances = distance_transform_edt(free, sampling=resolution)[1:-1, 1:-1]
        # An occupied cell and a candidate cell each extend half a diagonal from
        # their centres. This bound certifies the WHOLE candidate square as free.
        threshold = radius + math.sqrt(2.0) * resolution
        blocked = distances <= threshold + 1e-12
        return cls(raw.copy(), blocked, resolution, tuple(origin), yaw, radius)

    def world_to_grid(self, points):
        points = np.asarray(points, dtype=float)
        local = points - np.asarray(self.origin)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return local @ np.array([[c, -s], [s, c]]) / self.resolution

    def grid_to_world(self, points):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (np.asarray(points) * self.resolution) @ np.array([[c, s], [-s, c]]) + self.origin

    def _touches_only_free_cells(self, points):
        """Include both sides of grid lines and all four cells at grid corners."""
        if not np.isfinite(points).all():
            return False
        rounded = np.rint(points)
        boundary = np.abs(points - rounded) <= 1e-9
        cells = np.floor(np.where(boundary, rounded, points)).astype(np.int64)
        height, width = self.blocked.shape
        for dx, dy in ((0, 0), (-1, 0), (0, -1), (-1, -1)):
            mask = np.ones(len(points), dtype=bool)
            if dx:
                mask &= boundary[:, 0]
            if dy:
                mask &= boundary[:, 1]
            columns = cells[mask, 0] + dx
            rows = cells[mask, 1] + dy
            if ((columns < 0) | (columns >= width) | (rows < 0) | (rows >= height)).any():
                return False
            if self.blocked[rows, columns].any():
                return False
        return True

    def point_is_free(self, point):
        return self._touches_only_free_cells(self.world_to_grid([point]))

    def edge_is_free(self, start, end):
        """Exact grid supercover, including zero-length and grid-aligned segments.

        Split the segment at every vertical/horizontal grid line. Check all split
        points and one point inside every resulting interval. Each interval lies
        in a single cell (or on a grid line), so no intersected cell can be skipped.
        """
        segment = self.world_to_grid([start, end])
        if not np.isfinite(segment).all():
            return False
        height, width = self.blocked.shape
        if ((segment < 0).any() or (segment[:, 0] >= width).any()
                or (segment[:, 1] >= height).any()):
            return False
        first, last = segment
        delta = last - first
        crossings = [np.array([0.0, 1.0])]
        for axis in (0, 1):
            if abs(delta[axis]) <= 1e-15:
                continue
            low, high = sorted((first[axis], last[axis]))
            lines = np.arange(math.ceil(low), math.floor(high) + 1)
            times = (lines - first[axis]) / delta[axis]
            crossings.append(times[(times >= 0.0) & (times <= 1.0)])
        times = np.unique(np.concatenate(crossings))
        probes = np.concatenate((times, (times[:-1] + times[1:]) / 2.0))
        return self._touches_only_free_cells(first + probes[:, None] * delta)

    def sample_free(self, count, rng):
        rows, columns = np.nonzero(~self.blocked)
        if not len(rows):
            return np.empty((0, 2), dtype=float)
        indices = rng.integers(len(rows), size=count)
        # Uniform cells + uniform interior offsets gives uniform free-area samples.
        offsets = rng.uniform(1e-6, 1.0 - 1e-6, size=(count, 2))
        cells = np.column_stack((columns[indices], rows[indices]))
        return self.grid_to_world(cells + offsets)
