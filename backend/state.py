# backend/state.py
from dataclasses import dataclass, field
from threading import Lock
from typing import Any

from .research_core import ExperimentCapture

@dataclass
class SharedState:
    # put all experiment data in here
    core: ExperimentCapture = field(default_factory=ExperimentCapture)
    # keep a top-level lock if you want, but core has its own
    lock: Lock = field(default_factory=Lock)

    def seed(self, x: float, y: float, yaw: float):
        # delegate to core
        with self.core.lock:
            self.core.x = x
            self.core.y = y
            self.core.yaw = yaw
            self.core.path.clear()
            self.core.path.append((x, y))
