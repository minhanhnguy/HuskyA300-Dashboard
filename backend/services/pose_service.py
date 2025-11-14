# backend/services/pose_service.py
from __future__ import annotations
from typing import List
from ..models import PoseHistoryItem, PoseHistoryResponse, PoseHistoryMeta

def get_pose_history_service(core) -> PoseHistoryResponse:
    with core.lock:
        items: List[PoseHistoryItem] = [
            PoseHistoryItem(t=t, x=x, y=y, yaw=yaw) for (t, x, y, yaw) in core.pose_history
        ]
    return PoseHistoryResponse(pose_history=items)

def get_pose_history_meta_service(core) -> PoseHistoryMeta:
    with core.lock:
        n = len(core.pose_history)
        t0 = core.pose_history[0][0] if n else 0.0
        t1 = core.pose_history[-1][0] if n else 0.0
    return PoseHistoryMeta(count=n, t0=t0, t1=t1)
