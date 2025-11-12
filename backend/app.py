# backend/app.py
import asyncio
import json
import time
from typing import Any, Dict, List, Tuple

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .state import SharedState
from . import config as C
from .ros_worker import start_ros_in_thread, request_reverse_replay

app = FastAPI(title="Husky Dashboard")

# DEV-friendly CORS (works for HTTP & WS). Keep credentials False to allow wildcard origins.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

shared = SharedState()

# ---------------------- Lifecycle ----------------------
@app.on_event("startup")
async def on_startup():
    start_ros_in_thread(shared)

# ---------------------- Helpers ------------------------
def _snapshot_state() -> Dict[str, Any]:
    with shared.lock:
        path_pts = list(shared.path)
        pose = {"x": shared.x, "y": shared.y, "yaw": shared.yaw}
        v, w = shared.v, shared.w
        stats = {
            "rx_count": shared.stats.rx_count,
            "last_dt_ms": shared.stats.last_dt_ms,
            "hist_in": len(shared.history_in),
            "hist_out": len(shared.history_out),
            "pose_hist": len(shared.pose_history),
        }
        map_meta = shared.map_info
    return {"path": path_pts, "pose": pose, "v": v, "w": w, "stats": stats, "map_meta": map_meta}

# ---------------------- REST: Core ----------------------
@app.get("/api/v1/status")
async def status():
    return _snapshot_state()

@app.post("/api/v1/reset")
async def reset():
    with shared.lock:
        # Reset only the visual/integrated pose & path — keep histories and map.
        shared.seed(0.0, 0.0, 0.0)
    return {"ok": True}

@app.post("/api/v1/reverse_replay")
async def reverse_replay():
    ok = request_reverse_replay()
    return {"ok": bool(ok)}

# Pose history (timeline slider)
@app.get("/api/v1/pose_history")
async def pose_history():
    with shared.lock:
        ph = [{"t": t, "x": x, "y": y, "yaw": yaw} for (t, x, y, yaw) in shared.pose_history]
    return {"pose_history": ph}

@app.get("/api/v1/pose_history_meta")
async def pose_history_meta():
    with shared.lock:
        n = len(shared.pose_history)
        t0 = shared.pose_history[0][0] if n else 0.0
        t1 = shared.pose_history[-1][0] if n else 0.0
    return {"count": n, "t0": t0, "t1": t1}

# ---------------------- REST: Map -----------------------
@app.get("/api/v1/map_meta")
async def map_meta():
    with shared.lock:
        info = shared.map_info
    return info or {}

@app.get("/api/v1/map_full")
async def map_full():
    with shared.lock:
        g = shared.map_grid
        ver = shared.map_version
    if g is None:
        return {"version": ver, "data": []}
    # raw int8 flattened row-major
    return {"version": ver, "data": g.flatten(order='C').tolist()}

# ---------------------- WS: Path/Pose stream ------------
@app.websocket("/ws/stream")
async def ws_stream(ws: WebSocket):
    # Handy debug to see who connects
    print(f"[ws_stream] handshake from {getattr(ws.client, 'host', '?')}, "
          f"Origin={ws.headers.get('origin', '<?>')}")
    await ws.accept()
    try:
        # Initial snapshot
        snap = _snapshot_state()
        await ws.send_text(json.dumps({"type": "snapshot",
                                       "path": snap["path"],
                                       "pose": snap["pose"],
                                       "v": snap["v"], "w": snap["w"]}))
        last_idx = len(snap["path"])
        period = 1.0 / C.UI_HZ  # ~25 Hz

        while True:
            await asyncio.sleep(period)
            with shared.lock:
                n = len(shared.path)
                # send appended points (if any) plus latest pose
                if n > last_idx:
                    append = list(shared.path)[last_idx:n]
                    last_idx = n
                else:
                    append = []
                pose = {"x": shared.x, "y": shared.y, "yaw": shared.yaw}
            await ws.send_text(json.dumps({"type": "append",
                                           "append": append,
                                           "pose": pose}))
    except WebSocketDisconnect:
        return

# ---------------------- WS: Map patches @ 15 Hz ----------
@app.websocket("/ws/map")
async def ws_map(ws: WebSocket):
    print(f"[ws_map] handshake from {getattr(ws.client, 'host', '?')}, "
          f"Origin={ws.headers.get('origin', '<?>')}")
    await ws.accept()
    last_sent_version = -1
    try:
        period = 1.0 / C.MAP_WS_HZ  # 15 Hz

        # On connect: send meta + full
        with shared.lock:
            info = shared.map_info
            grid = shared.map_grid
        if info:
            await ws.send_text(json.dumps({"type": "map_meta", "meta": info}))
            if grid is not None:
                await ws.send_text(json.dumps({
                    "type": "map_full",
                    "version": info["version"],
                    "data": grid.flatten(order='C').tolist()
                }))
                last_sent_version = info["version"]

        # Loop: send patches; resend full if version changed
        while True:
            await asyncio.sleep(period)
            updates: List[Dict[str, Any]] = []

            with shared.lock:
                # Version bump => send fresh full map + meta
                if shared.map_info and shared.map_info["version"] != last_sent_version:
                    info = shared.map_info
                    grid = shared.map_grid
                    await ws.send_text(json.dumps({"type": "map_meta", "meta": info}))
                    if grid is not None:
                        await ws.send_text(json.dumps({
                            "type": "map_full",
                            "version": info["version"],
                            "data": grid.flatten(order='C').tolist()
                        }))
                        last_sent_version = info["version"]
                    # Old patches are irrelevant now
                    shared.map_patch_queue.clear()
                    continue

                # Drain patch queue
                while shared.map_patch_queue:
                    updates.append(shared.map_patch_queue.popleft())

            if updates:
                await ws.send_text(json.dumps({"type": "map_updates", "updates": updates}))

    except WebSocketDisconnect:
        return
