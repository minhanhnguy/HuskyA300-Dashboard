# backend/api/routes/status.py
"""
Status and control routes.
"""
from fastapi import APIRouter

from ..deps import get_shared, get_core
from ...core import config as C

router = APIRouter(tags=["status"])


@router.get("/status")
def status():
    """Get current system status."""
    return get_core().snapshot_for_ui()


@router.post("/reset")
def reset():
    """Reset the experiment state."""
    shared = get_shared()
    core = shared.core
    with core.lock:
        core.history_in.clear()
        core.history_out.clear()
        core.path.clear()
        core.x = core.y = core.yaw = 0.0
    return {"ok": True}


@router.post("/reverse_replay")
def reverse_replay():
    """Start reverse replay."""
    try:
        from ...ros import request_reverse_replay
        request_reverse_replay()
    except Exception:
        pass
    return {"ok": True}
