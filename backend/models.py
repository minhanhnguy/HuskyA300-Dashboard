# backend/models.py
from __future__ import annotations

from typing import List, Optional
from pydantic import BaseModel, ConfigDict, Field


# ======================
# Map models
# ======================

class MapOrigin(BaseModel):
  x: float
  y: float
  yaw: float


class MapMeta(BaseModel):
  """
  Metadata for an occupancy grid map.
  """
  model_config = ConfigDict(extra="forbid")

  width: int
  height: int
  resolution: float
  origin: MapOrigin
  version: int
  tile_size: int = Field(default=256)


class MapFullResponse(BaseModel):
  """
  Response for /api/v1/map_full:
  - version: map version
  - data: flattened int8 cells (row-major, origin at bottom-left in world)
  """
  version: int
  data: List[int]


class MapFullAtResponse(BaseModel):
  """
  Response for /api/v1/map_full_at:
  - meta: MapMeta
  - data: flattened int8 cells
  """
  meta: MapMeta
  data: List[int]


class MapDeltaPatch(BaseModel):
  """
  A patch for a sub-rectangle:
  - (x, y): cell coordinates of bottom-left corner
  - (w, h): dimensions
  - data: flattened int8 cells of size w*h
  """
  x: int
  y: int
  w: int
  h: int
  data: List[int]


class MapDeltaReset(BaseModel):
  """
  Full map reset payload (when version jumps).
  """
  meta: MapMeta
  data: List[int]


class MapDeltaResponse(BaseModel):
  """
  Response for /api/v1/map_delta:
  Either:
    - reset is set (full map image), or
    - patches is non-empty with the same version.
  """
  version: Optional[int] = None
  patches: Optional[List[MapDeltaPatch]] = None
  reset: Optional[MapDeltaReset] = None


# ======================
# Pose history
# ======================

class PoseHistoryItem(BaseModel):
  t: float
  x: float
  y: float
  yaw: float


class PoseHistoryResponse(BaseModel):
  pose_history: List[PoseHistoryItem]


class PoseHistoryMeta(BaseModel):
  count: int
  t0: float
  t1: float


# ======================
# Lidar scan (bag mode)
# ======================

class ScanAtResponse(BaseModel):
  """
  Lidar scan at (or near) a given bag time, in MAP frame.

  - t:      actual timestamp of the scan we used (seconds, bag time)
  - frame:  original scan frame_id
  - count:  number of (x,y) points
  - points: flattened [x0, y0, x1, y1, ...] in MAP frame (meters)
  """
  t: float
  frame: str
  count: int
  points: List[float]


# ======================
# cmd_vel stats (bag mode)
# ======================

class CmdInstant(BaseModel):
  """
  Instantaneous cmd_vel sample at (or just before) time t.

  - linear: (vx, vy, vz) in m/s
  - angular: (wx, wy, wz) in rad/s
  All in the robot's local frame.
  """
  t: float
  vx: float
  vy: float
  vz: float
  wx: float
  wy: float
  wz: float


class CmdPrefixStats(BaseModel):
  """
  Aggregate cmd_vel stats from bag start -> t1.

  - t0/t1: bag time span covered
  - sample_count: number of cmd samples up to t1 (inclusive)
  - max_vx_forward: max positive vx
  - max_vx_reverse: most negative vx
  - max_wz_abs: max absolute yaw rate
  - has_lateral: any vy or vz != 0
  - has_ang_xy: any wx or wy != 0
  """
  t0: float
  t1: float
  sample_count: int

  max_vx_forward: float
  max_vx_reverse: float
  max_wz_abs: float

  has_lateral: bool
  has_ang_xy: bool


class CmdStatsResponse(BaseModel):
  """
  Response for /api/v1/cmd_stats_at:

  - instant: cmd_vel at (or just before) t
  - prefix:  cumulative stats from bag start to t
  - bag_t0/bag_t1: overall cmd_vel time span in the bag
  """
  instant: Optional[CmdInstant] = None
  prefix: Optional[CmdPrefixStats] = None

  bag_t0: Optional[float] = None
  bag_t1: Optional[float] = None
