import asyncio
import json
from typing import Any, Dict, List, Optional

import numpy as np  # <<< added

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .state import SharedState
from .ros_worker import start_ros_in_thread, request_reverse_replay
from . import config as C

# Optional bag helpers (graceful fallback if file not present)
try:
    from .bag_replay import list_bag_files, replay_bag_in_thread
except Exception as _e:
    def list_bag_files():
        return []
    def replay_bag_in_thread(*args, **kwargs):
        raise FileNotFoundError("bag_replay.py not available")

app = FastAPI(title="Husky Dashboard")

# CORS for both HTTP and WS (allow all origins; credentials False to keep wildcard)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Single shared state object
shared = SharedState()


def get_core():
    """
    Some refactors store runtime under shared.core; older code used fields on shared directly.
    This helper returns the active object holding path/pose/map state.
    """
    return getattr(shared, "core", shared)


# ---------------------- Lifecycle ----------------------
@app.on_event("startup")
async def on_startup():
    # Start ROS2 node in background thread
    start_ros_in_thread(shared)


# ---------------------- Helpers ------------------------
def _snapshot_state() -> Dict[str, Any]:
    core = get_core()
    # If you introduced a research_core with snapshot_for_ui(), prefer it
    if hasattr(core, "snapshot_for_ui"):
        return core.snapshot_for_ui()

    # Fallback to direct fields
    with core.lock:
        path_pts = list(core.path)
        pose = {"x": core.x, "y": core.y, "yaw": core.yaw}
        v, w = core.v, core.w
        stats = {
            "rx_count": core.stats.rx_count,
            "last_dt_ms": core.stats.last_dt_ms,
            "hist_in": len(core.history_in),
            "hist_out": len(core.history_out),
            "pose_hist": len(core.pose_history),
        }
        map_meta = core.map_info
    return {
        "path": path_pts,
        "pose": pose,
        "v": v,
        "w": w,
        "stats": stats,
        "map_meta": map_meta,
    }


# ---------------------- REST: Core ----------------------
@app.get("/api/v1/status")
async def status():
    return _snapshot_state()


@app.post("/api/v1/reset")
async def reset():
    core = get_core()
    with core.lock:
        # Reset only the visual/integrated pose & path — keep histories and map.
        core.seed(0.0, 0.0, 0.0)
    return {"ok": True}


@app.post("/api/v1/reverse_replay")
async def reverse_replay():
    ok = request_reverse_replay()
    return {"ok": bool(ok)}


# ---------------------- REST: Pose history --------------
@app.get("/api/v1/pose_history")
async def pose_history():
    core = get_core()
    with core.lock:
        ph = [
            {"t": t, "x": x, "y": y, "yaw": yaw}
            for (t, x, y, yaw) in core.pose_history
        ]
    return {"pose_history": ph}


@app.get("/api/v1/pose_history_meta")
async def pose_history_meta():
    core = get_core()
    with core.lock:
        n = len(core.pose_history)
        t0 = core.pose_history[0][0] if n else 0.0
        t1 = core.pose_history[-1][0] if n else 0.0
    return {"count": n, "t0": t0, "t1": t1}


# ---------------------- REST: Map -----------------------
@app.get("/api/v1/map_meta")
async def map_meta():
    core = get_core()
    with core.lock:
        info = core.map_info
    return info or {}


@app.get("/api/v1/map_full")
async def map_full():
    core = get_core()
    with core.lock:
        g = core.map_grid
        ver = core.map_version
    if g is None:
        return {"version": ver, "data": []}
    # raw int8 flattened row-major
    return {"version": ver, "data": g.flatten(order="C").tolist()}


# Exact map snapshot at time t (for bag playback time-travel)
@app.get("/api/v1/map_full_at")
async def map_full_at(t: float = Query(..., description="Absolute bag time (seconds)")):
    """
    Returns the map state at exact bag time `t`.
    - If an index exists but no map exists at/before t, returns {} (not 404).
    - If no index exists, returns 404.
    """
    core = get_core()
    with core.lock:
        idx = getattr(core, "_bag_map_index", None)
    if idx is None:
        return JSONResponse(status_code=404, content={"error": "no bag index available"})

    snap = idx.snapshot_at(t)
    if not snap:
        return {}

    meta, grid = snap
    return {
        "meta": {
            "width": meta.width,
            "height": meta.height,
            "resolution": meta.resolution,
            "origin": {"x": meta.origin_x, "y": meta.origin_y, "yaw": meta.origin_yaw},
            "version": meta.version,
            "tile_size": int(getattr(C, "MAP_TILE_SIZE", 256)),
        },
        "data": grid.flatten(order="C").tolist()
    }


# ---------- NEW: Map delta between two bag times (for progressive refining in play) ----------
@app.get("/api/v1/map_delta")
async def map_delta(
    t0: float = Query(..., description="Start time (seconds, inclusive)"),
    t1: float = Query(..., description="End time (seconds, inclusive)"),
):
    """
    Returns minimal changes between map at t0 and map at t1.
    If a new full map version appeared in (t0, t1], we ask the client to reset by
    returning the full map at t1 (so the client replaces all tiles).
    Otherwise, we return tile-aligned patches for cells that changed.

    Response shapes:
      - {"reset": { "meta": {...}, "data": [int8...]} }
      - {"version": <int>, "patches": [ {x,y,w,h,data:[int8...]} , ... ] }
    """
    if t1 < t0:
        t0, t1 = t1, t0

    core = get_core()
    with core.lock:
        idx = getattr(core, "_bag_map_index", None)
    if idx is None:
        return JSONResponse(status_code=404, content={"error": "no bag index available"})

    snap1 = idx.snapshot_at(t1)
    if not snap1:
        return {}

    meta1, grid1 = snap1
    snap0 = idx.snapshot_at(t0)

    def meta_to_dict(m) -> Dict[str, Any]:
        return {
            "width": int(m.width),
            "height": int(m.height),
            "resolution": float(m.resolution),
            "origin": {"x": float(m.origin_x), "y": float(m.origin_y), "yaw": float(m.origin_yaw)},
            "version": int(m.version),
            "tile_size": int(getattr(C, "MAP_TILE_SIZE", 256)),
        }

    # If there was no map at t0 OR the version changed between t0 and t1,
    # instruct the client to reset to the full map at t1 (simplest & exact).
    if (not snap0) or (snap0[0].version != meta1.version):
        return {
            "reset": {
                "meta": meta_to_dict(meta1),
                "data": grid1.flatten(order="C").tolist()
            }
        }

    meta0, grid0 = snap0
    if grid0.shape != grid1.shape:
        # Safety: shapes changed -> reset
        return {
            "reset": {
                "meta": meta_to_dict(meta1),
                "data": grid1.flatten(order="C").tolist()
            }
        }

    # Compute changed cells then return tile-aligned patches for the changed tiles.
    changed = (grid0 != grid1)
    if not np.any(changed):
        return {"version": int(meta1.version), "patches": []}

    H, W = grid1.shape
    TS = int(getattr(C, "MAP_TILE_SIZE", 256))
    patches: List[Dict[str, Any]] = []

    # Iterate only tiles that have any change
    rows = (H + TS - 1) // TS
    cols = (W + TS - 1) // TS
    for r in range(rows):
        y0 = r * TS
        y1 = min(H, y0 + TS)
        for c in range(cols):
            x0 = c * TS
            x1 = min(W, x0 + TS)
            if np.any(changed[y0:y1, x0:x1]):
                sub = grid1[y0:y1, x0:x1]
                patches.append({
                    "x": int(x0),
                    "y": int(y0),
                    "w": int(x1 - x0),
                    "h": int(y1 - y0),
                    "data": sub.flatten(order="C").astype(np.int8).tolist(),
                })

    return {"version": int(meta1.version), "patches": patches}


# ---------------------- REST: Bag files -----------------
@app.get("/api/v1/bag/list")
async def bag_list():
    return {"bags": list_bag_files()}


@app.post("/api/v1/bag/play")
async def bag_play(payload: dict):
    """
    Start indexing a chosen bag (1x). Body: {"name": "<file>.mcap"}
    The indexer populates pose_history, path, and a time-travel map index.
    """
    name = payload.get("name")
    if not name:
        return JSONResponse(status_code=400, content={"error": "name is required"})
    try:
        replay_bag_in_thread(shared, name, speed=1.0)
    except FileNotFoundError:
        return JSONResponse(status_code=404, content={"error": "bag not found"})
    return {"ok": True}


# ---------------------- REST: Debug ---------------------
@app.get("/api/v1/debug/health")
async def debug_health():
    core = get_core()
    with core.lock:
        return {
            "pose_history_count": len(core.pose_history),
            "path_len": len(core.path),
            "has_bag_index": hasattr(core, "_bag_map_index") and (core._bag_map_index is not None),
            "map_info": core.map_info,
        }


# ---------------------- WS: Path/Pose stream ------------
@app.websocket("/ws/stream")
async def ws_stream(ws: WebSocket):
    """
    Periodically streams path appends + latest pose.
    Frontend uses this for live mode rendering.
    """
    await ws.accept()
    core = get_core()

    # Initial snapshot
    snap = _snapshot_state()
    await ws.send_text(json.dumps({
        "type": "snapshot",
        "path": snap.get("path", []),
        "pose": snap.get("pose", {"x": 0, "y": 0, "yaw": 0}),
        "v":    snap.get("v", 0.0),
        "w":    snap.get("w", 0.0),
    }))
    last_idx = len(snap.get("path", []))
    period = 1.0 / C.UI_HZ

    try:
        while True:
            await asyncio.sleep(period)
            with core.lock:
                n = len(core.path)
                if n > last_idx:
                    append = list(core.path)[last_idx:n]
                    last_idx = n
                else:
                    append = []
                pose = {"x": core.x, "y": core.y, "yaw": core.yaw}
            await ws.send_text(json.dumps({"type": "append", "append": append, "pose": pose}))
    except WebSocketDisconnect:
        return


# ---------------------- WS: Map stream ------------------
@app.websocket("/ws/map")
async def ws_map(ws: WebSocket):
    """
    Sends map meta/full on connect, then streams patches at ~MAP_WS_HZ.
    Version bump => resend full map + meta.
    """
    await ws.accept()
    core = get_core()
    last_sent_version = -1
    period = 1.0 / C.M
