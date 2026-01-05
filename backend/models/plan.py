# backend/models/plan.py
"""
Pydantic models for navigation plan and goal API responses.
"""
from __future__ import annotations

from typing import List
from pydantic import BaseModel


class PlanResponse(BaseModel):
    """
    Response for /api/v1/plan_at:
    - t: timestamp of the plan
    - poses: list of [x, y] tuples
    """
    t: float
    poses: List[List[float]]


class GoalResponse(BaseModel):
    """
    Response for /api/v1/goal_at:
    - t: timestamp of the goal
    - x: goal x
    - y: goal y
    - yaw: goal yaw
    """
    t: float
    x: float
    y: float
    yaw: float


class MidpointResponse(BaseModel):
    """
    Response for /api/v1/bag/midpoint:
    The 50% distance point calculated as (pose_history_distance + remaining_nav_path_distance) / 2.
    This matches the frontend's "Midpoint" calculation for realistic attack simulation.
    """
    t: float          # Timestamp at midpoint
    x: float          # X coordinate
    y: float          # Y coordinate
    yaw: float        # Orientation at midpoint
    pose_history_distance: float  # Distance traveled so far from pose history
    nav_path_distance: float      # Remaining distance on nav path
    total_distance: float         # pose_history_distance + nav_path_distance
    midpoint_distance: float      # total_distance / 2 (where midpoint is)
