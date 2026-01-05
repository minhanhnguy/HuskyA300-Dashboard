# backend/ros/lifecycle.py
"""
ROS lifecycle management - thread startup and control functions.
These are re-exported from ros/__init__.py for easy access.
"""
from typing import Any, Dict, Optional

# Import the node module to access the singleton
from . import node as _node_module


def start_ros_in_thread(shared):
    """Start ROS node in a background thread."""
    return _node_module.start_ros_in_thread(shared)


def request_reverse_replay(speed: float = None) -> bool:
    """Request reverse replay if node is available."""
    return _node_module.request_reverse_replay(speed)


def pause_ros():
    """Pause ROS node (for bag mode)."""
    _node_module.pause_ros()


def resume_ros():
    """Resume ROS node."""
    _node_module.resume_ros()


def publish_bag_snapshot(t: float, snapshot: Dict[str, Any]):
    """Publish bag snapshot to ROS topics."""
    _node_module.publish_bag_snapshot(t, snapshot)
