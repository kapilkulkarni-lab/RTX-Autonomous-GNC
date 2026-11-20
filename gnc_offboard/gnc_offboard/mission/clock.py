"""Clocks for the state machine: real (monotonic) for the node, fake for tests and sims."""
import time
from typing import Callable, List


class MonotonicClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, dt: float) -> None:
        time.sleep(dt)


class FakeClock:
    """Time moves only when a state sleeps. Hooks run after each sleep, so a test can feed
    inputs at chosen times. Raises once `limit_s` is passed, so a stuck state fails a test
    instead of hanging it."""

    def __init__(self, start: float = 1000.0, limit_s: float = 3600.0):
        self.t = start
        self.limit = start + limit_s
        self.hooks: List[Callable[[float], None]] = []

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt
        if self.t > self.limit:
            raise TimeoutError(f'FakeClock passed its limit at t={self.t:.2f}')
        for hook in list(self.hooks):
            hook(self.t)
