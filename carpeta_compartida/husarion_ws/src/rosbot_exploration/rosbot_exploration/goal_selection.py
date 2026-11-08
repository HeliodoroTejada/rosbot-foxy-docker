"""Retry memory and completion hysteresis, independent of ROS clocks."""
from dataclasses import dataclass
from collections import deque
import math

import numpy as np


@dataclass
class Attempt:
    x: float
    y: float
    failures: int
    until: float
    visits: int = 0
    context: object = None
    region_x: object = None
    region_y: object = None


def scene_signature(grid, costmap, x, y):
    """Sample a fixed metric patch, independent of map size/origin/resolution."""
    offsets = np.arange(-1.0, 1.001, 0.1)
    xx, yy = np.meshgrid(x + offsets, y + offsets)
    patches = []
    for source, threshold in ((grid, 25), (costmap, 80)):
        if source is None:
            patches.append(np.full(xx.shape, -2, dtype=np.int8))
            continue
        rr, cc = source.coordinates(xx, yy)
        rr, cc = np.floor(rr + 1e-7).astype(int), np.floor(cc + 1e-7).astype(int)
        h, w = source.data.shape
        inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        values = source.data[np.clip(rr, 0, h - 1), np.clip(cc, 0, w - 1)]
        patch = np.where(values < 0, -1, np.where(values > threshold, 1, 0)).astype(np.int8)
        patch[~inside] = -2
        patches.append(patch)
    return np.stack(patches)


def scene_improved(before, after, change_ratio=0.1):
    """Re-enable on meaningful new knowledge or net removal of obstacles.

    Timestamp changes and small wall shifts alone must not clear exclusions.
    """
    observed = (before >= -1) & (after >= -1)
    count = max(1, int(np.count_nonzero(observed)))
    discovered = np.count_nonzero(observed & (before < 0) & (after >= 0))
    old_obstacles = np.count_nonzero(observed & (before == 1))
    new_obstacles = np.count_nonzero(observed & (after == 1))
    return (discovered / count >= change_ratio or
            old_obstacles - new_obstacles >= max(3, math.ceil(old_obstacles * 0.2)))


class GoalMemory:
    def __init__(self, radius=0.5, retry_delay=30.0, max_attempts=3, revisit_delay=10.0,
                 blocked_retry_delay=180.0, visit_cooldown=90.0, scene_change_ratio=0.1,
                 frontier_radius=0.75):
        self.radius = radius
        self.retry_delay = retry_delay
        self.max_attempts = max_attempts
        self.revisit_delay = revisit_delay
        self.blocked_retry_delay = blocked_retry_delay
        self.visit_cooldown = visit_cooldown
        self.scene_change_ratio = scene_change_ratio
        self.frontier_radius = frontier_radius
        self.attempts = []
        self.trail = deque(maxlen=200)

    def clear(self):
        self.attempts.clear()
        self.trail.clear()

    def observe_pose(self, x, y, now):
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= 0.3:
            self.trail.append((x, y, now))

    def recent_positions(self, now):
        return [(x, y, math.exp(-(now - stamp) / 90.0)) for x, y, stamp in self.trail
                if now - stamp < 270.0]

    def entry(self, x, y):
        return next((a for a in self.attempts if math.hypot(a.x - x, a.y - y) <= self.radius), None)

    def allowed(self, frontier, now):
        entry = self.entry(frontier.x, frontier.y)
        if entry is not None and now < entry.until:
            return False
        return not any(math.hypot(frontier.region_x - a.region_x,
                                  frontier.region_y - a.region_y) <= self.frontier_radius
                       for a in self.attempts if a.failures and now < a.until and
                       a.region_x is not None and frontier.region_x is not None)

    def exclusions(self, now):
        return [(a.x, a.y, self.radius) for a in self.attempts if now < a.until]

    def frontier_exclusions(self, now):
        return [(a.region_x, a.region_y, self.frontier_radius) for a in self.attempts
                if a.failures and now < a.until and a.region_x is not None]

    def update_scene(self, grid, costmap, now):
        resets = 0
        for entry in self.attempts:
            if not entry.failures or now >= entry.until or entry.context is None:
                continue
            x = entry.region_x if entry.region_x is not None else entry.x
            y = entry.region_y if entry.region_y is not None else entry.y
            current = scene_signature(grid, costmap, x, y)
            if scene_improved(entry.context, current, self.scene_change_ratio):
                entry.failures = 0
                entry.until = now
                entry.context = current
                resets += 1
        return resets

    def record(self, frontier, now, success=False, grid=None, costmap=None):
        entry = self.entry(frontier.x, frontier.y)
        if entry is None and frontier.region_x is not None:
            entry = next((a for a in self.attempts if a.region_x is not None and
                          math.hypot(a.region_x - frontier.region_x, a.region_y - frontier.region_y)
                          <= self.frontier_radius), None)
        if entry is None:
            entry = Attempt(frontier.x, frontier.y, 0, 0,
                            region_x=frontier.region_x, region_y=frontier.region_y)
            self.attempts.append(entry)
        if success:
            entry.failures = 0
            entry.visits += 1
            entry.until = (now + self.visit_cooldown if entry.visits >= self.max_attempts
                           else now + self.revisit_delay)
        else:
            entry.failures += 1
            entry.until = now + (self.blocked_retry_delay if entry.failures >= self.max_attempts
                                 else self.retry_delay * 2 ** (entry.failures - 1))
        if grid is not None:
            x = entry.region_x if entry.region_x is not None else entry.x
            y = entry.region_y if entry.region_y is not None else entry.y
            entry.context = scene_signature(grid, costmap, x, y)

    def temporarily_blocked(self, now):
        return any(now < a.until < float('inf') for a in self.attempts)


class CompletionGate:
    def __init__(self, confirmations=3, delay=5.0):
        self.confirmations = confirmations
        self.delay = delay
        self.reset()

    def reset(self):
        self.revision = None
        self.count = 0
        self.since = None

    def update(self, empty, revision, now):
        if not empty:
            self.reset()
            return False
        if self.since is None:
            self.since = now
        if revision != self.revision:
            self.revision = revision
            self.count += 1
        return self.count >= self.confirmations and now - self.since >= self.delay
