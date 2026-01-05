# backend/core/state.py
"""
SharedState: Top-level container for all experiment data.
"""
from dataclasses import dataclass, field
from threading import Lock

from .experiment import ExperimentCapture


@dataclass
class SharedState:
    """
    Shared state container accessible across the application.
    All experiment data lives in self.core.
    """
    # All experiment data lives here
    core: ExperimentCapture = field(default_factory=ExperimentCapture)
    # Top-level lock if needed (but core has its own)
    lock: Lock = field(default_factory=Lock)

    def seed(self, x: float, y: float, yaw: float):
        """Reset the pose and path to a seed position."""
        with self.core.lock:
            self.core.x = x
            self.core.y = y
            self.core.yaw = yaw
            self.core.path.clear()
            self.core.path.append((x, y))
