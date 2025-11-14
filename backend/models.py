# backend/models.py
from __future__ import annotations
from typing import List, Optional
from pydantic import BaseModel, Field, ConfigDict


# ---------- Map ----------
class MapOrigin(BaseModel):
    x: float
    y: float
    yaw: float

class MapMeta(BaseModel):
    model_config = ConfigDict(extra="forbid")
    width: int
    height: int
    resolution: float
    origin: MapOrigin
    version: int
    tile_size: int = Field(default=256)

class MapFullResponse(BaseModel):
    # /api/v1/map_full
    version: int
    data: List[int]

class MapFullAtResponse(BaseModel):
    # /api/v1/map_full_at
    meta: MapMeta
    data: List[int]

class MapDeltaPatch(BaseModel):
    x: int
    y: int
    w: int
    h: int
    data: List[int]

class MapDeltaReset(BaseModel):
    meta: MapMeta
    data: List[int]

class MapDeltaResponse(BaseModel):
    # Either a reset, or a set of patches for an existing version
    version: Optional[int] = None
    patches: Optional[List[MapDeltaPatch]] = None
    reset: Optional[MapDeltaReset] = None


# ---------- Pose history ----------
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
