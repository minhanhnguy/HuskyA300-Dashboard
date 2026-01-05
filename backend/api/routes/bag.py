# backend/api/routes/bag.py
"""
Bag file management routes.
"""
import math
from fastapi import APIRouter

from ..deps import get_shared, get_core

router = APIRouter(prefix="/bag", tags=["bag"])


@router.get("/list")
def bag_list():
    """List available bag files."""
    try:
        from ...bag import list_bag_files
        return list_bag_files()
    except Exception as e:
        return {"error": str(e)}


@router.post("/play")
def bag_play(payload: dict):
    """
    Start indexing a chosen bag (1x). Body: {"name": "<file>.mcap", "spoofed": false}
    If spoofed=true, loads from spoofed_bags/ directory instead.
    """
    name = payload.get("name")
    spoofed = payload.get("spoofed", False)
    if not name:
        return {"error": "name required"}
    try:
        from ...bag import replay_bag_in_thread
        from ...ros import pause_ros
        # Pause live ROS processing
        try:
            pause_ros()
        except Exception:
            pass
        replay_bag_in_thread(get_shared(), name, speed=1.0, spoofed=spoofed)
        return {"ok": True, "name": name, "spoofed": spoofed}
    except Exception as e:
        return {"error": str(e)}


@router.post("/spoof")
def bag_spoof(payload: dict):
    """
    Generate a spoofed bag from an existing bag.
    Body: {"name": "<file>.mcap"}
    """
    name = payload.get("name")
    if not name:
        return {"error": "name required"}
    try:
        from ...bag import create_spoofed_bag
        output_path = create_spoofed_bag(name)
        return {"ok": True, "output_path": output_path}
    except Exception as e:
        import traceback
        traceback.print_exc()
        return {"error": str(e)}


@router.get("/spoofed_list")
def spoofed_bag_list():
    """List available spoofed bag files."""
    try:
        from ...bag import list_spoofed_bags
        return list_spoofed_bags()
    except Exception as e:
        return {"error": str(e)}


@router.get("/midpoint")
def bag_midpoint(t: float = None):
    """
    Get the 50% distance point based on pose history + remaining nav path.
    If t is provided, estimates midpoint as of that time.
    """
    core = get_core()
    with core.lock:
        ph = list(core.pose_history)
        plan_index = getattr(core, "_bag_plan_index", None)
    
    if not ph:
        return {"error": "No pose history available"}
    
    # Calculate distance traveled from pose history
    pose_dist = 0.0
    for i in range(1, len(ph)):
        dx = ph[i][1] - ph[i-1][1]
        dy = ph[i][2] - ph[i-1][2]
        pose_dist += math.sqrt(dx*dx + dy*dy)
    
    # Get nav path distance
    nav_dist = 0.0
    if plan_index is not None:
        target_t = t if t is not None else ph[-1][0]
        plan = plan_index.nearest(target_t)
        if plan and len(plan) > 1:
            for i in range(1, len(plan)):
                dx = plan[i][0] - plan[i-1][0]
                dy = plan[i][1] - plan[i-1][1]
                nav_dist += math.sqrt(dx*dx + dy*dy)
    
    total_dist = pose_dist + nav_dist
    midpoint_dist = total_dist / 2.0
    
    # Find pose at midpoint distance
    cumulative = 0.0
    midpoint_pose = ph[-1]
    for i in range(1, len(ph)):
        dx = ph[i][1] - ph[i-1][1]
        dy = ph[i][2] - ph[i-1][2]
        seg_dist = math.sqrt(dx*dx + dy*dy)
        if cumulative + seg_dist >= midpoint_dist:
            midpoint_pose = ph[i]
            break
        cumulative += seg_dist
    
    return {
        "t": midpoint_pose[0],
        "x": midpoint_pose[1],
        "y": midpoint_pose[2],
        "yaw": midpoint_pose[3],
        "pose_history_distance": pose_dist,
        "nav_path_distance": nav_dist,
        "total_distance": total_dist,
        "midpoint_distance": midpoint_dist,
    }
