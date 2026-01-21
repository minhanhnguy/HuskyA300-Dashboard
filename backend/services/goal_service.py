from __future__ import annotations
from typing import Optional

def goal_at_service(core, t: float) -> Optional[dict]:
    with core.lock:
        goal_index = getattr(core, "_bag_goal_index", None)
    if goal_index is None:
        return {"t": t, "x": 0, "y": 0, "yaw": 0}
    
    goal = goal_index.latest_at(t)
    if goal is None:
        return {"t": t, "x": 0, "y": 0, "yaw": 0}
    return {"t": t, "x": goal[0], "y": goal[1], "yaw": goal[2]}
