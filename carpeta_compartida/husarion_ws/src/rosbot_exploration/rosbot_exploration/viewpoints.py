"""Visibility estimates and oriented footprint checks, independent of ROS."""
import math

import numpy as np


def angle_distance(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def footprint_is_free(grid, x, y, yaw, length=0.28, width=0.32,
                      padding=0.02, free_threshold=25):
    """Test every grid square intersecting the padded, rotated rectangle.

    Separating-axis tests include rectangle interiors and corner contacts;
    testing only vertices or edges could miss an obstacle inside the footprint.
    Unknown and off-map cells are not free.
    """
    row, col = grid.coordinates(x, y)
    half_l, half_w = length / 2 + padding, width / 2 + padding
    c, s = math.cos(yaw - grid.origin_yaw), math.sin(yaw - grid.origin_yaw)
    extent_x = abs(c) * half_l + abs(s) * half_w
    extent_y = abs(s) * half_l + abs(c) * half_w
    r0, r1 = math.floor(row - extent_y / grid.resolution), math.floor(row + extent_y / grid.resolution)
    c0, c1 = math.floor(col - extent_x / grid.resolution), math.floor(col + extent_x / grid.resolution)
    if r0 < 0 or c0 < 0 or r1 >= grid.data.shape[0] or c1 >= grid.data.shape[1]:
        return False
    rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
    dx, dy = (cc + 0.5 - col) * grid.resolution, (rr + 0.5 - row) * grid.resolution
    cell_projection = grid.resolution * 0.5 * (abs(c) + abs(s))
    overlaps = ((abs(dx) <= extent_x + grid.resolution / 2) &
                (abs(dy) <= extent_y + grid.resolution / 2) &
                (abs(dx * c + dy * s) <= half_l + cell_projection) &
                (abs(-dx * s + dy * c) <= half_w + cell_projection))
    values = grid.data[r0:r1 + 1, c0:c1 + 1]
    return bool(np.all((values[overlaps] >= 0) & (values[overlaps] <= free_threshold)))


def visible_unknown_area(grid, x, y, radius=3.0, rays=72, free_threshold=25):
    """Approximate unknown visible area in m² with bounded polar ray casting.

    Known obstacles and map bounds stop each ray. Unknown is potential visible
    space, not a promise that unseen walls do not exist. Half-cell radial steps
    retain thin mapped walls; polar area weights avoid resolution-dependent
    counts of repeated ray samples.
    """
    step = grid.resolution / 2
    distance = np.arange(step / 2, radius, step)
    if not len(distance):
        return 0.0
    angles = np.arange(rays) * (2 * math.pi / rays)
    wx = x + np.cos(angles[:, None]) * distance[None, :]
    wy = y + np.sin(angles[:, None]) * distance[None, :]
    rr, cc = grid.coordinates(wx, wy)
    rr, cc = np.floor(rr).astype(int), np.floor(cc).astype(int)
    height, width = grid.data.shape
    inside = (rr >= 0) & (rr < height) & (cc >= 0) & (cc < width)
    values = grid.data[np.clip(rr, 0, height - 1), np.clip(cc, 0, width - 1)]
    blocked = ~inside | (values > free_threshold)
    visible = ~np.logical_or.accumulate(blocked, axis=1)
    area = (2 * math.pi / rays) * distance * step
    return float(np.sum(visible * (values < 0) * area[None, :]))


def turn_cost(robot, robot_yaw, x, y, goal_yaw):
    if robot_yaw is None:
        return 0.0
    bearing = math.atan2(y - robot[1], x - robot[0])
    return angle_distance(bearing, robot_yaw) + 0.5 * angle_distance(goal_yaw, bearing)


def revisit_cost(x, y, recent_positions, radius=0.8):
    """Recent trail samples carry decayed weights prepared by GoalMemory."""
    return max((weight * math.exp(-((x - px) ** 2 + (y - py) ** 2) / radius ** 2)
                for px, py, weight in recent_positions), default=0.0)


def utility_cost(distance, gain, turn=0.0, revisit=0.0, distance_weight=1.0,
                 gain_weight=0.5, turn_weight=0.35, revisit_weight=1.0):
    """Travel/observation cost divided by sublinear expected information gain."""
    effort = 0.8 + distance_weight * distance + turn_weight * turn + revisit_weight * revisit
    return effort / max(gain, 0.25) ** gain_weight
