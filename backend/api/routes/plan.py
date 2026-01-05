# backend/api/routes/plan.py
"""
Plan and goal routes.
"""
from fastapi import APIRouter, Query

from ..deps import get_core

router = APIRouter(tags=["plan"])


@router.get("/plan_at")
def plan_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
):
    """Get navigation plan at a specific bag time."""
    core = get_core()
    with core.lock:
        plan_index = getattr(core, "_bag_plan_index", None)
    if plan_index is None:
        return {"t": t, "poses": []}
    
    poses = plan_index.nearest(t)
    if poses is None:
        return {"t": t, "poses": []}
    return {"t": t, "poses": [[x, y] for (x, y) in poses]}


@router.get("/goal_at")
def goal_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
):
    """Get navigation goal at a specific bag time."""
    core = get_core()
    with core.lock:
        goal_index = getattr(core, "_bag_goal_index", None)
    if goal_index is None:
        return {"t": t, "x": 0, "y": 0, "yaw": 0}
    
    goal = goal_index.latest_at(t)
    if goal is None:
        return {"t": t, "x": 0, "y": 0, "yaw": 0}
    return {"t": t, "x": goal[0], "y": goal[1], "yaw": goal[2]}


@router.get("/robot_description")
def robot_description():
    """Get robot URDF description."""
    core = get_core()
    with core.lock:
        desc = getattr(core, "robot_description", None)
    if desc is None:
        return {"urdf": None}
    return {"urdf": desc}
