# backend/app.py
"""
Husky Dashboard - FastAPI Application

This is the main entry point for the backend API.
All route handlers have been moved to backend/api/routes/.
"""
import os
import shutil

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

# Import core components
from .core import SharedState
from .api.deps import set_shared, get_shared, get_core
from .api.routes import router as api_router

# Bag directories for cleanup
try:
    from .bag import SPOOFED_BAG_DIR
except ImportError:
    SPOOFED_BAG_DIR = None

app = FastAPI(title="Husky Dashboard")

# CORS for both HTTP and WS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Create shared state
shared = SharedState()
set_shared(shared)


# ---------------------- Lifecycle ----------------------

@app.on_event("startup")
def on_startup():
    """Start ROS node on application startup."""
    try:
        from .ros import start_ros_in_thread
        start_ros_in_thread(shared)
    except Exception as e:
        print(f"[startup] ROS node failed to start: {e}")


@app.on_event("shutdown")
def on_shutdown():
    """Clean up spoofed bag files when server shuts down."""
    if SPOOFED_BAG_DIR and os.path.isdir(SPOOFED_BAG_DIR):
        try:
            shutil.rmtree(SPOOFED_BAG_DIR)
            print(f"[shutdown] Cleaned up {SPOOFED_BAG_DIR}")
        except Exception as e:
            print(f"[shutdown] Failed to clean up spoofed bags: {e}")


# ---------------------- Include API Routes ----------------------

app.include_router(api_router)


# ---------------------- WebSocket Endpoints ----------------------

@app.websocket("/ws/stream")
async def ws_stream(websocket: WebSocket):
    """
    WebSocket for streaming pose/path updates to the frontend (live mode).
    """
    await websocket.accept()
    import asyncio
    import json
    
    core = get_core()
    
    # Send initial snapshot
    snap = core.snapshot_for_ui()
    await websocket.send_text(json.dumps({"type": "snapshot", **snap}))
    
    last_path_len = len(core.path)
    
    try:
        while True:
            await asyncio.sleep(0.04)  # ~25 Hz
            with core.lock:
                path_len = len(core.path)
                if path_len > last_path_len:
                    new_pts = list(core.path)[last_path_len:]
                    pose = {"x": core.x, "y": core.y, "yaw": core.yaw}
                    last_path_len = path_len
                    await websocket.send_text(json.dumps({
                        "type": "append",
                        "append": new_pts,
                        "pose": pose,
                    }))
    except WebSocketDisconnect:
        pass


@app.websocket("/ws/map")
async def ws_map(websocket: WebSocket):
    """
    WebSocket for streaming map updates to the frontend (live mode).
    """
    await websocket.accept()
    import asyncio
    import json
    
    core = get_core()
    last_version = -1
    
    try:
        while True:
            await asyncio.sleep(0.1)  # ~10 Hz
            with core.lock:
                if core.map_info is None:
                    continue
                    
                current_version = core.map_version
                if current_version != last_version:
                    # Full map update
                    await websocket.send_text(json.dumps({
                        "type": "map_meta",
                        "meta": core.map_info,
                    }))
                    if core.map_grid is not None:
                        await websocket.send_text(json.dumps({
                            "type": "map_full",
                            "version": current_version,
                            "data": core.map_grid.flatten(order="C").tolist(),
                        }))
                    last_version = current_version
                    
                # Send patches if any
                if core.map_patch_queue:
                    patches = list(core.map_patch_queue)
                    core.map_patch_queue.clear()
                    await websocket.send_text(json.dumps({
                        "type": "map_updates",
                        "updates": patches,
                    }))
    except WebSocketDisconnect:
        pass


# ---------------------- Publish Snapshot Endpoint ----------------------

@app.post("/api/v1/publish_snapshot")
async def publish_snapshot(payload: dict):
    """
    Publish a snapshot to ROS topics for RViz visualization.
    """
    t = payload.get("t", 0.0)
    core = get_core()
    
    try:
        from .services.snapshot_service import get_bag_snapshot_at_service
        from .ros import publish_bag_snapshot
        
        snapshot = get_bag_snapshot_at_service(core, t)
        if snapshot:
            publish_bag_snapshot(t, snapshot)
            return {"ok": True}
        return {"ok": False, "error": "No snapshot data"}
    except Exception as e:
        return {"ok": False, "error": str(e)}
