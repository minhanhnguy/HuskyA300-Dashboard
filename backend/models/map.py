# backend/models/map.py
"""
Pydantic models for map-related API responses.
"""
from __future__ import annotations

from typing import List, Optional
from pydantic import BaseModel, ConfigDict, Field


class MapOrigin(BaseModel):
    """Map origin in world coordinates."""
    x: float
    y: float
    yaw: float


class MapMeta(BaseModel):
    """Metadata for an occupancy grid map."""
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
    """Full map reset payload (when version jumps)."""
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
