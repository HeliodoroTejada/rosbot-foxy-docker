"""Retry memory and completion hysteresis, independent of ROS clocks."""
from dataclasses import dataclass
import math


@dataclass
class Attempt:
    x: float
    y: float
    failures: int
    until: float
    visits: int = 0


class GoalMemory:
    def __init__(self, radius=0.5, retry_delay=30.0, max_attempts=3, revisit_delay=10.0):
        self.radius = radius
        self.retry_delay = retry_delay
        self.max_attempts = max_attempts
        self.revisit_delay = revisit_delay
        self.attempts = []

    def entry(self, x, y):
        return next((a for a in self.attempts if math.hypot(a.x - x, a.y - y) <= self.radius), None)

    def allowed(self, frontier, now):
        entry = self.entry(frontier.x, frontier.y)
        return entry is None or now >= entry.until

    def exclusions(self, now):
        return [(a.x, a.y, self.radius) for a in self.attempts if now < a.until]

    def record(self, frontier, now, success=False):
        entry = self.entry(frontier.x, frontier.y)
        if entry is None:
            entry = Attempt(frontier.x, frontier.y, 0, 0)
            self.attempts.append(entry)
        if success:
            entry.failures = 0
            entry.visits += 1
            entry.until = (float('inf') if entry.visits >= self.max_attempts
                           else now + self.revisit_delay)
        else:
            entry.failures += 1
            entry.until = (float('inf') if entry.failures >= self.max_attempts
                           else now + self.retry_delay)

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
