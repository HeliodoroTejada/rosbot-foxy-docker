"""ROS-independent grid geometry and frontier extraction (metres, not cell counts)."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .viewpoints import footprint_is_free, revisit_cost, turn_cost, utility_cost, visible_unknown_area


@dataclass
class Grid:
    data: np.ndarray
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def __post_init__(self):
        if self.data.ndim != 2 or not self.data.size:
            raise ValueError('Empty or non-rectangular occupancy grid')
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Grid resolution must be positive')
        if not all(math.isfinite(v) for v in
                   (self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError('Non-finite grid origin')

    def cell(self, x, y):
        row, col = self.coordinates(x, y)
        return math.floor(row), math.floor(col)

    def coordinates(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return ((-s * dx + c * dy) / self.resolution,
                (c * dx + s * dy) / self.resolution)

    def world(self, row, col):
        x, y = (col + 0.5) * self.resolution, (row + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * x - s * y, self.origin_y + s * x + c * y

    def contains(self, row, col):
        return 0 <= row < self.data.shape[0] and 0 <= col < self.data.shape[1]


@dataclass
class Frontier:
    cells: np.ndarray
    x: float
    y: float
    yaw: float
    distance: float
    size: float
    score: float
    information_gain: float = 0.0
    turn: float = 0.0
    revisit: float = 0.0
    region_x: object = None
    region_y: object = None


@dataclass
class SearchResult:
    frontiers: list
    count: int
    safe: np.ndarray
    excluded: bool = False


def adjacent(mask):
    """Four-connected neighbours, without wrapping across map boundaries."""
    result = np.zeros_like(mask)
    result[1:] |= mask[:-1]
    result[:-1] |= mask[1:]
    result[:, 1:] |= mask[:, :-1]
    result[:, :-1] |= mask[:, 1:]
    return result


def clearance_mask(free, resolution, radius):
    """Require a circular footprint entirely inside known free space.

    Include half a cell diagonal: occupied cells represent areas, not points.
    Unknown and off-map space are obstacles for this geometric check.
    """
    reach = int(math.ceil(radius / resolution + math.sqrt(0.5)))
    padded = np.pad(free, reach, mode='constant', constant_values=False)
    safe = free.copy()
    h, w = free.shape
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            # Minimum distance from the centre to the shifted cell square.
            distance = math.hypot(max(abs(dr) - 0.5, 0), max(abs(dc) - 0.5, 0))
            if distance * resolution <= radius:
                safe &= padded[reach + dr:reach + dr + h, reach + dc:reach + dc + w]
    return safe


def distances(mask, start):
    """Four-connected geodesic distance in cells, -1 for disconnected space."""
    row, col = start
    h, w = mask.shape
    result = [-1] * (h * w)
    if not (0 <= row < h and 0 <= col < w and mask[row, col]):
        return np.asarray(result, dtype=np.int32).reshape(mask.shape)
    # Python integer/list indexing avoids millions of slow NumPy scalar
    # operations on large SLAM maps. The resulting distances are identical.
    traversable = mask.ravel().tolist()
    first = int(row) * w + int(col)
    result[first] = 0
    queue = deque([first])
    while queue:
        index = queue.popleft()
        value = result[index] + 1
        col = index % w
        for neighbour in (index - w if index >= w else -1,
                          index + w if index < (h - 1) * w else -1,
                          index - 1 if col else -1,
                          index + 1 if col + 1 < w else -1):
            if neighbour >= 0 and traversable[neighbour] and result[neighbour] < 0:
                result[neighbour] = value
                queue.append(neighbour)
    return np.asarray(result, dtype=np.int32).reshape(mask.shape)


def components(mask):
    """Group boundary cells with eight-connectivity."""
    remaining = mask.copy()
    h, w = mask.shape
    for row, col in np.argwhere(mask):
        if not remaining[row, col]:
            continue
        queue = deque([(int(row), int(col))])
        remaining[row, col] = False
        group = []
        while queue:
            r, c = queue.popleft()
            group.append((r, c))
            for nr in range(max(0, r - 1), min(h, r + 2)):
                for nc in range(max(0, c - 1), min(w, c + 2)):
                    if remaining[nr, nc]:
                        remaining[nr, nc] = False
                        queue.append((nr, nc))
        yield np.asarray(group, dtype=np.int32)


def search(grid, robot, *, robot_radius=0.25, free_threshold=25,
           min_frontier_size=0.35, goal_standoff=0.45, max_standoff=0.9,
           distance_weight=1.0, gain_weight=0.5, start_tolerance=0.3,
           min_goal_distance=0.5, excluded_positions=(), costmap=None,
           costmap_max_cost=80, robot_yaw=None, recent_positions=(),
           information_radius=3.0, information_rays=72, information_candidates=12,
           candidates_per_frontier=3, turn_weight=0.35, revisit_weight=1.0,
           use_rectangular_footprint=False, footprint_length=0.28,
           footprint_width=0.32, footprint_padding=0.02, excluded_frontiers=()):
    """Find viewpoints on the reachable, footprint-cleared side of frontiers."""
    free = (grid.data >= 0) & (grid.data <= free_threshold)
    raw_distance = distances(free, grid.cell(*robot))
    if not np.any(raw_distance >= 0):
        raise ValueError('Robot is outside known free map space')
    # A disk inscribed in the padded rectangle is a conservative necessary
    # centre-connectivity test, not a complete orientation-aware path planner.
    # Check the full oriented footprint at each viewpoint; Nav2 checks routes.
    connectivity_radius = (min(footprint_length, footprint_width) / 2 + footprint_padding
                           if use_rectangular_footprint else robot_radius)
    safe = clearance_mask(free, grid.resolution, connectivity_radius)
    seeds = np.argwhere(safe & (raw_distance >= 0) &
                        (raw_distance * grid.resolution <= start_tolerance))
    if not len(seeds):
        raise ValueError('No footprint-cleared starting cell near the robot')
    seed = min(seeds, key=lambda p: int(raw_distance[tuple(p)]))
    travel = distances(safe, tuple(seed))
    frontier_mask = free & (raw_distance >= 0) & adjacent(grid.data < 0)
    frontiers, count, excluded = [], 0, False
    h, w = free.shape
    reach = int(math.ceil(max_standoff / grid.resolution))
    for cells in components(frontier_mask):
        size = len(cells) * grid.resolution
        if size < min_frontier_size:
            continue
        count += 1
        candidates = []
        # Include the nearest boundary first; sample long boundaries uniformly.
        nearest = int(np.argmin(raw_distance[cells[:, 0], cells[:, 1]]))
        samples = np.unique(np.r_[nearest, np.linspace(0, len(cells) - 1, 24).astype(int)])
        for index in samples:
            fr, fc = cells[index]
            sample_x, sample_y = grid.world(fr, fc)
            if any(math.hypot(sample_x - ex, sample_y - ey) <= radius
                   for ex, ey, radius in excluded_frontiers):
                excluded = True
                continue
            r0, r1 = max(0, fr - reach), min(h, fr + reach + 1)
            c0, c1 = max(0, fc - reach), min(w, fc + reach + 1)
            rr, cc = np.mgrid[r0:r1, c0:c1]
            separation = np.hypot(rr - fr, cc - fc) * grid.resolution
            eligible = ((travel[r0:r1, c0:c1] >= 0) &
                        (separation >= connectivity_radius) & (separation <= max_standoff))
            wx, wy = grid.world(rr, cc)
            eligible &= np.hypot(wx - robot[0], wy - robot[1]) >= min_goal_distance
            if costmap is not None:
                cr, ccost = costmap.coordinates(wx, wy)
                cr, ccost = np.floor(cr).astype(int), np.floor(ccost).astype(int)
                ch, cw = costmap.data.shape
                inside = (cr >= 0) & (cr < ch) & (ccost >= 0) & (ccost < cw)
                costs = costmap.data[np.clip(cr, 0, ch - 1), np.clip(ccost, 0, cw - 1)]
                eligible &= inside & (costs >= 0) & (costs <= costmap_max_cost)
            before_exclusion = np.any(eligible)
            for ex, ey, radius in excluded_positions:
                eligible &= np.hypot(wx - ex, wy - ey) > radius
            if before_exclusion and not np.any(eligible):
                excluded = True
            if not np.any(eligible):
                continue
            merit = travel[r0:r1, c0:c1] * grid.resolution + 2 * abs(separation - goal_standoff)
            merit[~eligible] = np.inf
            # Bound expensive oriented checks around each boundary sample.
            for flat in np.argsort(merit, axis=None)[:12]:
                local = np.unravel_index(flat, merit.shape)
                if not math.isfinite(float(merit[local])):
                    break
                row, col = local[0] + r0, local[1] + c0
                x, y = grid.world(row, col)
                fx, fy = grid.world(fr, fc)
                yaw = math.atan2(fy - y, fx - x)
                if use_rectangular_footprint and not footprint_is_free(
                        grid, x, y, yaw, footprint_length, footprint_width,
                        footprint_padding, free_threshold):
                    continue
                distance = (int(travel[row, col]) + int(raw_distance[tuple(seed)])) * grid.resolution
                turning = turn_cost(robot, robot_yaw, x, y, yaw)
                revisit = revisit_cost(x, y, recent_positions)
                preliminary = (distance_weight * distance + turn_weight * turning +
                               revisit_weight * revisit) / math.sqrt(1 + min(size, 6.0))
                candidates.append(Frontier(cells, x, y, yaw, distance, size,
                                            preliminary, turn=turning, revisit=revisit,
                                            region_x=fx, region_y=fy))
                break
        selected = []
        for candidate in sorted(candidates, key=lambda f: f.score):
            if all(math.hypot(candidate.x - f.x, candidate.y - f.y) >= 0.6 for f in selected):
                selected.append(candidate)
                if len(selected) >= candidates_per_frontier:
                    break
        frontiers.extend(selected)
    # Evaluate only the most promising viewpoints to bound CPU use. Gain is an
    # estimate of visible unknown area, not the length of a frontier component.
    finalists = sorted(frontiers, key=lambda f: f.score)[:information_candidates]
    for frontier in finalists:
        frontier.information_gain = visible_unknown_area(
            grid, frontier.x, frontier.y, information_radius, information_rays, free_threshold)
        frontier.score = utility_cost(frontier.distance, frontier.information_gain,
                                      frontier.turn, frontier.revisit, distance_weight,
                                      gain_weight, turn_weight, revisit_weight)
    return SearchResult(sorted(finalists, key=lambda f: f.score), count, safe, excluded)


def path_is_safe(points, grid, safe):
    """Supercover traversal checks every crossed cell, including corner contacts."""
    if not points:
        return False
    previous = points[0]
    for point in points:
        if not all(math.isfinite(v) for v in (*previous, *point)):
            return False
        start_r, start_c = grid.coordinates(*previous)
        end_r, end_c = grid.coordinates(*point)
        row, col = math.floor(start_r), math.floor(start_c)
        er, ec = math.floor(end_r), math.floor(end_c)
        if not grid.contains(row, col) or not grid.contains(er, ec) or not safe[row, col]:
            return False
        dr, dc = end_r - start_r, end_c - start_c
        sr, sc = (1 if dr > 0 else -1), (1 if dc > 0 else -1)
        tr = ((row + (dr > 0) - start_r) / dr) if dr else float('inf')
        tc = ((col + (dc > 0) - start_c) / dc) if dc else float('inf')
        while (row, col) != (er, ec):
            if abs(tr - tc) <= 1e-12:
                if (not grid.contains(row + sr, col) or not safe[row + sr, col] or
                        not grid.contains(row, col + sc) or not safe[row, col + sc]):
                    return False
                row, col = row + sr, col + sc
                tr += abs(1 / dr)
                tc += abs(1 / dc)
            elif tr < tc:
                row += sr
                tr += abs(1 / dr)
            else:
                col += sc
                tc += abs(1 / dc)
            if not grid.contains(row, col) or not safe[row, col]:
                return False
        previous = point
    return True
