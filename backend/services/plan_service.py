from __future__ import annotations
from typing import Optional, List, Tuple

def plan_at_service(core, t: float) -> Optional[dict]:
    with core.lock:
        plan_index = getattr(core, "_bag_plan_index", None)
    if plan_index is None:
        return {"t": t, "poses": []}
    
    poses = plan_index.nearest(t)
    if poses is None:
        return {"t": t, "poses": []}
    return {"t": t, "poses": [[x, y] for (x, y) in poses]}
