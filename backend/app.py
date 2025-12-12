# backend/app.py
import asyncio
import json
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .state import SharedState

from .ros_worker import start_ros_in_thread, request_reverse_replay, pause_ros, resume_ros, publish_bag_snapshot
from . import config as C

from .models import (
    PoseHistoryResponse,
    PoseHistoryMeta,
    MapFullResponse,
    MapFullAtResponse,
    MapDeltaResponse,
    ScanAtResponse,
    ScanAtResponse,
    CmdStatsResponse,
    PlanResponse,
    GoalResponse,
)
from .services.pose_service import (
    get_pose_history_service,
    get_pose_history_meta_service,
)
from .services.map_service import (
    map_full_at_service,
    map_delta_service,
)
from .services.scan_service import get_scan_at_service
from .services.cmd_service import get_cmd_stats_at_service
from .services.costmap_service import (
    global_costmap_full_at_service,
    local_costmap_full_at_service,
)
from .services.snapshot_service import get_bag_snapshot_at_service

# Optional bag helpers (graceful fallback if file not present)
try:
    from .bag_replay import list_bag_files, replay_bag_in_thread
except Exception:

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
    """Return the object holding path/pose/map state (shared.core in your design)."""
    return getattr(shared, "core", shared)


# ---------------------- Lifecycle ----------------------
@app.on_event("startup")
async def on_startup():
    start_ros_in_thread(shared)


# ---------------------- Helpers ------------------------
def _snapshot_state() -> Dict[str, Any]:
    core = get_core()
    if hasattr(core, "snapshot_for_ui"):
        return core.snapshot_for_ui()

    # Fallback path (older code paths)
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
        core.x = 0.0
        core.y = 0.0
        core.yaw = 0.0
        core.path.clear()
        core.path.append((0.0, 0.0))
    resume_ros()
    return {"ok": True}


@app.post("/api/v1/reverse_replay")
async def reverse_replay():
    ok = request_reverse_replay()
    return {"ok": bool(ok)}


# ---------------------- REST: Pose history --------------
@app.get("/api/v1/pose_history", response_model=PoseHistoryResponse)
async def pose_history():
    return get_pose_history_service(get_core())


@app.get("/api/v1/pose_history_meta", response_model=PoseHistoryMeta)
async def pose_history_meta():
    return get_pose_history_meta_service(get_core())


# ---------------------- REST: Map -----------------------
@app.get("/api/v1/map_meta")
async def map_meta():
    core = get_core()
    with core.lock:
        info = core.map_info
    return info or {}


@app.get("/api/v1/map_full", response_model=MapFullResponse)
async def map_full():
    core = get_core()
    with core.lock:
        g = core.map_grid
        ver = core.map_version
    if g is None:
        return MapFullResponse(version=int(ver), data=[])
    return MapFullResponse(version=int(ver), data=g.flatten(order="C").tolist())


@app.get("/api/v1/map_full_at", response_model=MapFullAtResponse)
async def map_full_at(t: float = Query(..., description="Absolute bag time (seconds)")):
    core = get_core()
    out = map_full_at_service(core, t)
    if out is None:
        # Distinguish "no index" vs "no map before t"
        with core.lock:
            idx = getattr(core, "_bag_map_index", None)
        if idx is None:
            return JSONResponse(
                status_code=404, content={"error": "no bag index available"}
            )
        return JSONResponse(status_code=200, content={})
    return out


@app.get("/api/v1/map_delta", response_model=MapDeltaResponse)
async def map_delta(
    t0: float = Query(..., description="Start time (seconds, inclusive)"),
    t1: float = Query(..., description="End time (seconds, inclusive)"),
):
    core = get_core()
    out = map_delta_service(core, t0, t1)
    if out is None:
        return JSONResponse(
            status_code=404, content={"error": "no bag index available"}
        )
    return out


# ---------------------- REST: Costmaps ------------------
@app.get("/api/v1/costmap/global/full_at", response_model=MapFullAtResponse)
async def costmap_global_full_at(t: float = Query(..., description="Absolute bag time")):
    core = get_core()
    out = global_costmap_full_at_service(core, t)
    if out is None:
        return JSONResponse(status_code=404, content={"error": "no data"})
    return out


@app.get("/api/v1/costmap/local/full_at", response_model=MapFullAtResponse)
async def costmap_local_full_at(t: float = Query(..., description="Absolute bag time")):
    core = get_core()
    out = local_costmap_full_at_service(core, t)
    if out is None:
        return JSONResponse(status_code=404, content={"error": "no data"})
    return out


# ---------------------- REST: Lidar scan (bag mode) -----
@app.get("/api/v1/scan_at", response_model=ScanAtResponse)
async def scan_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
    decim: int = Query(
        1, ge=1, description="Decimation factor over beam index (1 = all beams)"
    ),
    max_points: int = Query(
        12000, ge=1, description="Max number of points to return"
    ),
    tol: float = Query(
        0.05, ge=0.0, description="Time tolerance in seconds for nearest scan"
    ),
):
    """
    Return lidar scan points in MAP frame nearest to time t (bag mode).
    """
    core = get_core()
    return get_scan_at_service(core, t=t, decim=decim, max_points=max_points, tol=tol)


# ---------------------- REST: cmd_vel stats (bag mode) --
@app.get("/api/v1/cmd_stats_at", response_model=CmdStatsResponse)
async def cmd_stats_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
):
    core = get_core()
    return get_cmd_stats_at_service(core, t)


# ---------------------- REST: Plan/Goal (bag mode) ------
@app.get("/api/v1/plan_at", response_model=PlanResponse)
async def plan_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
):
    core = get_core()
    with core.lock:
        idx = getattr(core, "_bag_plan_index", None)
    
    if idx is None:
        return JSONResponse(status_code=404, content={"error": "no plan index"})
    
    poses = idx.nearest(t)
    if poses is None:
        return JSONResponse(status_code=404, content={"error": "no plan found"})
    
    # poses is list of (x, y)
    return PlanResponse(t=t, poses=[[p[0], p[1]] for p in poses])


@app.get("/api/v1/goal_at", response_model=GoalResponse)
async def goal_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
):
    core = get_core()
    with core.lock:
        idx = getattr(core, "_bag_goal_index", None)
    
    if idx is None:
        return JSONResponse(status_code=404, content={"error": "no goal index"})
    
    # latest_at returns (x, y, yaw)
    res = idx.latest_at(t)
    if res is None:
        return JSONResponse(status_code=404, content={"error": "no goal found"})
    
    return GoalResponse(t=t, x=res[0], y=res[1], yaw=res[2])


@app.get("/api/v1/robot_description")
async def robot_description():
    core = get_core()
    with core.lock:
        desc = getattr(core, "robot_description", None)
    
    if desc is None:
        return JSONResponse(status_code=404, content={"error": "no robot description found"})
    
    return {"urdf": desc}


# ---------------------- REST: Bag files -----------------
@app.get("/api/v1/bag/list")
async def bag_list():
    return {"bags": list_bag_files()}


@app.post("/api/v1/bag/play")
async def bag_play(payload: dict):
    """
    Start indexing a chosen bag (1x). Body: {"name": "<file>.mcap"}
    """
    name = payload.get("name")
    if not name:
        return JSONResponse(
            status_code=400, content={"error": "name is required"}
        )
    try:
        pause_ros()
        replay_bag_in_thread(shared, name, speed=1.0)
    except FileNotFoundError:
        return JSONResponse(
            status_code=404, content={"error": "bag not found"}
        )
    return {"ok": True}


@app.post("/api/v1/bag/publish_at")
async def bag_publish_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
):
    """
    Publish bag data at time t to ROS topics for RViz visualization.
    """
    core = get_core()
    snapshot = get_bag_snapshot_at_service(core, t)
    publish_bag_snapshot(t, snapshot)
    return {"ok": True}


# ---------------------- REST: Debug ---------------------
@app.get("/api/v1/debug/health")
async def debug_health():
    core = get_core()
    with core.lock:
        return {
            "pose_history_count": len(core.pose_history),
            "path_len": len(core.path),
            "has_bag_index": hasattr(core, "_bag_map_index")
            and (core._bag_map_index is not None),
            "map_info": core.map_info,
        }


# ---------------------- WS: Path/Pose stream ------------
@app.websocket("/ws/stream")
async def ws_stream(ws: WebSocket):
    """Periodically streams path appends + latest pose (unchanged API)."""
    await ws.accept()
    core = get_core()

    # Initial snapshot
    snap = _snapshot_state()
    await ws.send_text(
        json.dumps(
            {
                "type": "snapshot",
                "path": snap.get("path", []),
                "pose": snap.get("pose", {"x": 0, "y": 0, "yaw": 0}),
                "v": snap.get("v", 0.0),
                "w": snap.get("w", 0.0),
            }
        )
    )
    last_idx = len(snap.get("path", []))
    period = 1.0 / C.UI_HZ

    try:
        while True:
            await asyncio.sleep(period)
            with core.lock:
                n = len(core.path)
                append = list(core.path)[last_idx:n] if n > last_idx else []
                last_idx = max(last_idx, n)
                pose = {"x": core.x, "y": core.y, "yaw": core.yaw}
            await ws.send_text(
                json.dumps({"type": "append", "append": append, "pose": pose})
            )
    except WebSocketDisconnect:
        return


# ---------------------- WS: Map stream ------------------
@app.websocket("/ws/map")
async def ws_map(ws: WebSocket):
    """Live map stream (unchanged API)."""
    await ws.accept()
    core = get_core()
    last_sent_version = -1
    period = 1.0 / C.MAP_WS_HZ

    # On connect: send meta + full
    with core.lock:
        info = core.map_info
        grid = core.map_grid
    if info:
        await ws.send_text(json.dumps({"type": "map_meta", "meta": info}))
        if grid is not None:
            await ws.send_text(
                json.dumps(
                    {
                        "type": "map_full",
                        "version": info["version"],
                        "data": grid.flatten(order="C").tolist(),
                    }
                )
            )
            last_sent_version = info["version"]

    try:
        while True:
            await asyncio.sleep(period)
            updates: List[Dict[str, Any]] = []

            with core.lock:
                # Version bump => send fresh full map + meta
                if core.map_info and core.map_info["version"] != last_sent_version:
                    info = core.map_info
                    grid = core.map_grid
                    await ws.send_text(
                        json.dumps({"type": "map_meta", "meta": info})
                    )
                    if grid is not None:
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "map_full",
                                    "version": info["version"],
                                    "data": grid.flatten(order="C").tolist(),
                                }
                            )
                        )
                        last_sent_version = info["version"]
                    core.map_patch_queue.clear()
                    continue

                while core.map_patch_queue:
                    updates.append(core.map_patch_queue.popleft())

            if updates:
                await ws.send_text(
                    json.dumps({"type": "map_updates", "updates": updates})
                )
    except WebSocketDisconnect:
        return
