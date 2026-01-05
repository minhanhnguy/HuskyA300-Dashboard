# backend/models/cmd.py
"""
Pydantic models for cmd_vel statistics API responses.
"""
from __future__ import annotations

from typing import Optional
from pydantic import BaseModel


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
    avg_vx: float
    max_vy_abs: float
    avg_vy_abs: float
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
