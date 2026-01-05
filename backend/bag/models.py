# backend/bag/models.py
"""
Data classes for bag indexing and replay.
"""
from dataclasses import dataclass
from typing import List


@dataclass
class StampedTransform:
    """A stamped 3D transform (for TF messages)."""
    t: float
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class MapMeta:
    """Metadata for a map version in the bag."""
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    version: int
    frame_id: str = "map"


@dataclass
class StampedSE2:
    """A stamped 2D pose (x, y, yaw)."""
    t: float
    x: float
    y: float
    yaw: float


@dataclass
class Scan:
    """A single lidar scan."""
    t: float
    frame: str
    angle_min: float
    angle_inc: float
    range_min: float
    range_max: float
    ranges: List[float]
