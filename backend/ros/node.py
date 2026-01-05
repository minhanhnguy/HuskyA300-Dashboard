"""
ROS-side adapter for the Husky dashboard.

PURPOSE OF THIS FILE (high level):
- Talk to ROS2 (subscribe to /cmd_vel, /odom, /map, ...).
- Convert ROS messages into plain Python values.
- Hand those values to the *research core* (shared.core).
- Expose a small “reverse replay” publisher so the UI can play back commands.

IMPORTANT:
- This file is *plumbing*. The “interesting / researchy” part lives in
  backend/research_core.py and is accessed through shared.core.
- Keep this thin: each callback gets a ROS msg → extracts numbers → calls core.
"""

import math
import threading
import time
from typing import Any, Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, PoseStamped
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String as StringMsg
from rosgraph_msgs.msg import Clock

# Import TF types unconditionally; we’ll just not use them if disabled.
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from ..core.state import SharedState
from ..utils.math import clip, wrap_pi
from ..core import config as C

# we keep a process-global pointer so /api/v1/reverse_replay can trigger it
_NODE_SINGLETON: Optional["CmdVelNode"] = None


class CmdVelNode(Node):
    """
    One ROS2 node that:
    - subscribes to IN and OUT cmd_vel (as TwistStamped),
    - subscribes to map and map patches,
    - subscribes ONLY to the exact odom/filtered topic for pose,
    - does NOT use TF and does NOT integrate cmd_vel for pose (fallback disabled),
    - periodically prints heartbeat,
    - can publish reverse-replay cmd_vel.

    It does NOT keep long histories itself — it passes everything to shared.core.
    """

    def __init__(self, shared: SharedState):
        super().__init__("cmdvel_dash")

        # shared contains the research core (ExperimentCapture)
        self.shared = shared
        self.core = shared.core  # shorter alias

        # Pose source is forced to odom; integrator is effectively disabled
        self.pose_mode = "odom"

        # Paused flag (for bag mode)
        self.paused = False

        # flags to know what we managed to latch on
        self._tf_ok = False
        self._odom_ok = False
        self._odom_topic: Optional[str] = None

        # time for pose-history sampling
        self._last_pose_sample_t = 0.0
        # time for integrator sampling (kept but no-op due to pose_mode)
        self._last_int_pose_sample_t = 0.0

        # remember whether we ever saw a cmd_vel
        self._first_rx = False

        # to smooth v, w when integrating (not used now, but kept)
        self._v_f = 0.0
        self._w_f = 0.0

        # try to run under simulation time (gazebo, rosbag2, etc.)
        from rclpy.parameter import Parameter
        try:
            self.set_parameters([Parameter(name="use_sim_time", value=True)])
        except Exception:
            pass

        # ------------------------------------------------------------------
        # INITIALIZE core pose/path so the UI doesn't start empty
        # ------------------------------------------------------------------
        with self.core.lock:
            if C.SEED_MODE == "manual":
                self.core.x = C.SEED_X
                self.core.y = C.SEED_Y
                self.core.yaw = C.SEED_YAW
            else:
                self.core.x = 0.0
                self.core.y = 0.0
                self.core.yaw = 0.0
            self.core.path.clear()
            self.core.path.append((self.core.x, self.core.y))

        # ------------------------------------------------------------------
        # ROS SUBSCRIPTIONS
        # ------------------------------------------------------------------
        # 1) Upstream cmd_vel (what the operator/planner commanded)
        self.sub_in = self.create_subscription(
            TwistStamped, C.RECORD_IN_TOPIC, self.on_cmd_in, 50
        )
        # 2) Downstream / executed cmd_vel (what the robot actually used)
        self.sub_out = self.create_subscription(
            TwistStamped, C.RECORD_OUT_TOPIC, self.on_cmd_out, 50
        )
        # 3) Full map from SLAM
        self.sub_map = self.create_subscription(
            OccupancyGrid, C.MAP_TOPIC, self.on_map_full, 1
        )
        # 4) Map updates (incremental patches)
        self.sub_map_up = self.create_subscription(
            OccupancyGridUpdate, C.MAP_UPDATES_TOPIC, self.on_map_patch, 50
        )

        self.get_logger().info(
            f"[cmdvel_dash] Record IN {C.RECORD_IN_TOPIC}, OUT {C.RECORD_OUT_TOPIC}"
        )
        self.get_logger().info(
            f"[map] Subscribed to {C.MAP_TOPIC} and {C.MAP_UPDATES_TOPIC}"
        )

        # 5) Global Costmap
        self.sub_gc = self.create_subscription(
            OccupancyGrid, "/a300_0000/global_costmap/costmap", self.on_global_costmap, 1
        )
        # 6) Local Costmap
        self.sub_lc = self.create_subscription(
            OccupancyGrid, "/a300_0000/local_costmap/costmap", self.on_local_costmap, 1
        )

        # ------------------------------------------------------------------
        # PUBLISHER for reverse-replay (we will publish TwistStamped)
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(TwistStamped, C.PUBLISH_TOPIC, qos)
        self.get_logger().info(
            f"[cmdvel_dash] Replay publisher -> {C.PUBLISH_TOPIC}"
        )

        # ------------------------------------------------------------------
        # PUBLISHERS for bag playback (RViz visualization)
        # ------------------------------------------------------------------
        ns = getattr(C, "NAMESPACE", "/a300_0000")
        if ns.endswith("/"):
            ns = ns[:-1]
        if not ns.startswith("/"):
            ns = "/" + ns

        self.pub_clock = self.create_publisher(Clock, "/clock", 10) # Clock is always global

        # TF: Publish to both global and namespaced to be safe, or just namespaced if that's what RViz wants.
        # Given the user's list has /a300_0000/tf, we should publish there.
        self.pub_tf = self.create_publisher(TFMessage, f"{ns}/tf", 10)
        self.pub_tf_static = self.create_publisher(TFMessage, f"{ns}/tf_static", qos_profile=QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

        # MAP: Needs TRANSIENT_LOCAL durability for RViz
        map_qos = QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(OccupancyGrid, f"{ns}/map", qos_profile=map_qos)
        self.pub_gc = self.create_publisher(OccupancyGrid, f"{ns}/global_costmap/costmap", qos_profile=map_qos)
        self.pub_lc = self.create_publisher(OccupancyGrid, f"{ns}/local_costmap/costmap", qos_profile=map_qos)
        
        self.pub_scan = self.create_publisher(LaserScan, f"{ns}/sensors/lidar2d_0/scan", 10)
        self.pub_odom = self.create_publisher(Odometry, f"{ns}/odom", 10)
        self.pub_plan = self.create_publisher(Path, f"{ns}/plan", 10)
        self.pub_goal = self.create_publisher(PoseStamped, f"{ns}/goal_pose", 10)
        self.pub_robot_desc = self.create_publisher(StringMsg, f"{ns}/robot_description", qos_profile=QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        
        self._last_robot_desc = None

        # ------------------------------------------------------------------
        # TIMERS
        # ------------------------------------------------------------------
        # integrator timer remains but will early-out because pose_mode="odom"
        self.timer = self.create_timer(1.0 / C.INT_HZ, self.integrate_step)
        # heartbeat: 1 Hz print
        self.hb = self.create_timer(1.0, self._heartbeat)

        # ------------------------------------------------------------------
        # TF & ODOM helpers
        # ------------------------------------------------------------------
        # We disable TF completely per config.
        if getattr(C, "USE_TF_POSE", False):
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tf_timer = self.create_timer(1.0 / C.UI_HZ, self._poll_tf_pose)
        else:
            self.tf_buffer = None
            self.tf_listener = None
            self.tf_timer = None

        # Only probe the exact odom/filtered topic; no auto-discovery.
        self.odom_probe_timer = self.create_timer(0.5, self._probe_odom_once)

    # ======================================================================
    # CMD_VEL HANDLERS
    # ======================================================================

    def on_cmd_in(self, msg: TwistStamped):
        if self.paused:
            return
        now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 if msg.header.stamp else \
                self.get_clock().now().nanoseconds * 1e-9
        v = clip(msg.twist.linear.x, -C.V_MAX, C.V_MAX)
        w = clip(msg.twist.angular.z, -C.W_MAX, C.W_MAX)
        self.core.record_cmd_in(v, w, now_s)
        if not self._first_rx:
            self._first_rx = True
            self.get_logger().info(f"[cmdvel_dash] First IN rx: v={v:.3f}, w={w:.3f}")

    def on_cmd_out(self, msg: TwistStamped):
        if self.paused:
            return
        now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 if msg.header.stamp else \
                self.get_clock().now().nanoseconds * 1e-9
        v = clip(msg.twist.linear.x, -C.V_MAX, C.V_MAX)
        w = clip(msg.twist.angular.z, -C.W_MAX, C.W_MAX)
        self.core.record_cmd_out(v, w, now_s)

    # ======================================================================
    # MAP HANDLERS
    # ======================================================================

    def on_map_full(self, msg: OccupancyGrid):
        if self.paused:
            return
        info = msg.info
        w, h = info.width, info.height

        arr = (
            np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
        )

        q = info.origin.orientation
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        yaw0 = math.atan2(t3, t4)

        meta = {
            "width": int(w),
            "height": int(h),
            "resolution": float(info.resolution),
            "origin": {
                "x": float(info.origin.position.x),
                "y": float(info.origin.position.y),
                "yaw": float(yaw0),
            },
        }

        self.core.set_full_map(arr, meta, C.MAP_TILE_SIZE)
        self.get_logger().info(
            f"[map] full map v{self.core.map_version}: {w}x{h} @ {info.resolution} m"
        )

    def on_map_patch(self, up: OccupancyGridUpdate):
        if self.paused:
            return
        patch = {
            "x": int(up.x),
            "y": int(up.y),
            "w": int(up.width),
            "h": int(up.height),
            "data": list(up.data),
        }
        self.core.apply_map_patch(patch)

    def on_global_costmap(self, msg: OccupancyGrid):
        if self.paused:
            return
        info = msg.info
        w, h = info.width, info.height
        arr = (
            np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
        )
        q = info.origin.orientation
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        yaw0 = math.atan2(t3, t4)

        meta = {
            "width": int(w),
            "height": int(h),
            "resolution": float(info.resolution),
            "origin": {
                "x": float(info.origin.position.x),
                "y": float(info.origin.position.y),
                "yaw": float(yaw0),
            },
        }
        self.core.set_global_costmap(arr, meta)

    def on_local_costmap(self, msg: OccupancyGrid):
        if self.paused:
            return
        info = msg.info
        w, h = info.width, info.height
        arr = (
            np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
        )
        q = info.origin.orientation
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        yaw0 = math.atan2(t3, t4)

        meta = {
            "width": int(w),
            "height": int(h),
            "resolution": float(info.resolution),
            "origin": {
                "x": float(info.origin.position.x),
                "y": float(info.origin.position.y),
                "yaw": float(yaw0),
            },
        }
        self.core.set_local_costmap(arr, meta)

    # ======================================================================
    # ODOM (forced)
    # ======================================================================

    def _probe_odom_once(self):
        """
        Called every 0.5s until we find the EXACT Odometry topic:
        /a300_0000/platform/odom/filtered
        No fallback probing to other topics.
        """
        if self._odom_topic is not None:
            return
        if "odom" not in C.POSE_PREF_ORDER:
            return

        names_types = dict(self.get_topic_names_and_types())
        exact = C.ODOM_CANDIDATES[0] if C.ODOM_CANDIDATES else None
        if exact and exact in names_types and any(
            t.endswith("nav_msgs/msg/Odometry") for t in names_types[exact]
        ):
            self.create_subscription(Odometry, exact, self.on_odom, 10)
            self._odom_topic = exact
            self.get_logger().info(f"[odom] Subscribed to {exact}")
        else:
            # Log once every few tries to avoid spam
            pass

    def on_odom(self, msg: Odometry):
        """
        Use Odometry as the only pose source.
        Timestamps are from the message header.
        """
        if self.paused:
            return
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        yaw = math.atan2(t3, t4)

        # Use message header stamp (requested)
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # write pose into core
        with self.core.lock:
            self.core.x = px
            self.core.y = py
            self.core.yaw = yaw
            # keep path sparse
            if (
                not self.core.path
                or (px - self.core.path[-1][0]) ** 2 + (py - self.core.path[-1][1]) ** 2
                >= C.MOVE_EPS ** 2
            ):
                self.core.path.append((px, py))
            # also store in timeline, but only at ~UI_HZ
            if (t_now - self._last_pose_sample_t) >= (1.0 / C.UI_HZ):
                self.core.pose_history.append((t_now, px, py, yaw))
                self._last_pose_sample_t = t_now

        self._odom_ok = True
        self.pose_mode = "odom"

    # ======================================================================
    # TF (disabled)
    # ======================================================================

    def _poll_tf_pose(self):
        # Disabled by config (C.USE_TF_POSE=False)
        return

    # ======================================================================
    # INTEGRATOR (kept but disabled via pose_mode)
    # ======================================================================

    def _smooth(self, v: float, w: float):
        if not C.SMOOTHING:
            return v, w
        self._v_f = C.ALPHA * self._v_f + (1.0 - C.ALPHA) * v
        self._w_f = C.ALPHA * self._w_f + (1.0 - C.ALPHA) * w
        return self._v_f, self._w_f

    def integrate_step(self):
        # Fallback disabled: do nothing unless explicitly enabled
        if self.pose_mode in ("tf", "odom"):
            return
        # (No-op)

    # ======================================================================
    # HEARTBEAT
    # ======================================================================

    def _heartbeat(self):
        with self.core.lock:
            npts = len(self.core.path)
            hin = len(self.core.history_in)
            hout = len(self.core.history_out)
            ph = len(self.core.pose_history)
            mver = self.core.map_version
            mgrid = self.core.map_grid
            msz = None if mgrid is None else (mgrid.shape[1], mgrid.shape[0])
            lv = self.core.last_v
            lw = self.core.last_w

        self.get_logger().info(
            f"[hb] v={lv:+.2f}, w={lw:+.2f}, path={npts}, "
            f"hist_in={hin}, hist_out={hout}, pose_hist={ph}, "
            f"map v{mver} size={msz}, pose_mode={self.pose_mode}, "
            f"tf_ok={self._tf_ok}, odom_ok={self._odom_ok}, odom_topic={self._odom_topic}"
        )

    # ======================================================================
    # REVERSE REPLAY
    # ======================================================================

    def start_reverse_replay(self, speed: float = None) -> bool:
        from ..core import config as C2

        if speed is None:
            speed = C2.REPLAY_SPEED

        with self.core.lock:
            if self.core.replaying:
                self.get_logger().warn("Replay already running")
                return False

            use_out = (
                self.core.replay_source == "out"
                and len(self.core.history_out) > 1
            )
            hist = (
                list(self.core.history_out)
                if use_out
                else list(self.core.history_in)
            )
            self.core.replaying = True
            self.core.recording = False

        if len(hist) < 2:
            self.get_logger().warn("Not enough history to replay")
            with self.core.lock:
                self.core.replaying = False
                self.core.recording = True
            return False

        times = [t for (_, _, t) in hist]
        dts = [max(0.0, times[i + 1] - times[i]) for i in range(len(times) - 1)]

        def _thread():
            self.get_logger().info(
                f"Reverse replay using "
                f"{'EXECUTED' if hist is self.core.history_out else 'INPUT'} stream: "
                f"{len(hist)} samples, speed={speed}x"
            )
            try:
                for i in range(len(hist) - 1, -1, -1):
                    v, w, _t = hist[i]
                    msg = TwistStamped()
                    if C2.PUBLISH_STAMP_MODE == "now":
                        msg.header.stamp = self.get_clock().now().to_msg()
                    else:
                        msg.header.stamp = RosTime(sec=0, nanosec=0)
                    msg.header.frame_id = C2.PUBLISH_FRAME_ID
                    msg.twist.linear.x = -v
                    msg.twist.angular.z = -w
                    self.pub.publish(msg)
                    if i > 0:
                        time.sleep(dts[i - 1] / max(1e-6, speed))
            finally:
                with self.core.lock:
                    self.core.replaying = False
                    self.core.recording = True
                self.get_logger().info("Reverse replay finished.")

        threading.Thread(target=_thread, daemon=True).start()
        return True

    # ======================================================================
    # PAUSE / RESUME (for bag mode)
    # ======================================================================

    def pause(self):
        """Stop processing incoming ROS messages (so bag replay is exclusive)."""
        self.paused = True
        self.get_logger().info("[cmdvel_dash] Paused live ROS listeners.")

    def resume(self):
        """Resume processing incoming ROS messages."""
        self.paused = False
        self.get_logger().info("[cmdvel_dash] Resumed live ROS listeners.")

    # ======================================================================
    # BAG SNAPSHOT PUBLISHING
    # ======================================================================

    def publish_snapshot(self, t: float, snapshot: Dict[str, Any]):
        """
        Publish a snapshot of bag data to ROS topics.
        snapshot dict keys: 'tf', 'tf_static', 'map', 'scan', 'odom', 'plan', 'goal', 'robot_description'
        """
        # 1. Clock
        clock_msg = Clock()
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        clock_msg.clock = RosTime(sec=sec, nanosec=nanosec)
        self.pub_clock.publish(clock_msg)

        # 2. TF
        if "tf" in snapshot and snapshot["tf"]:
            # snapshot['tf'] is a list of TFMessage objects or dicts?
            # Assuming we construct TFMessage here or receive it.
            # Let's assume snapshot['tf'] is a list of transform dicts or objects we can convert.
            # Actually, bag_replay stores them as _StampedSE2 or similar.
            # We need to convert them to TransformStamped.
            # For simplicity, let's assume the snapshot service prepares the ROS messages or we do it here.
            # Doing it here is better if we want to keep service pure.
            # But converting _StampedSE2 to TransformStamped requires some math.
            pass
        
        # Actually, let's assume the snapshot contains the raw data and we convert it here.
        # OR, simpler: The snapshot contains the *nearest* message from the bag.
        # But bag_replay deserializes them into internal structs.
        # We might need to reconstruct ROS messages.

        # Let's look at what bag_replay has.
        # It has _StampedSE2 for TF.
        # It has numpy arrays for Map.
        # It has _Scan for Scan.
        
        # We need to convert these back to ROS messages.
        
        # TF
        if "tf_list" in snapshot:
            tf_msg = TFMessage()
            for tf_data in snapshot["tf_list"]:
                # tf_data: {child_frame_id, header: {frame_id, stamp}, transform: {translation, rotation}}
                # This seems too complex to reconstruct if we only have _StampedSE2.
                # _StampedSE2 only has x, y, yaw.
                # But bag_replay also stores `tf_pairs` which are _StampedSE2.
                # We might be missing the full 3D transform if we only use _StampedSE2.
                # However, for 2D nav, SE2 is often enough.
                pass

        # Let's try to pass pre-constructed ROS messages (or dicts that look like them) from the service.
        # That way this node just publishes.
        
        if "scan" in snapshot and snapshot["scan"]:
            self.pub_scan.publish(snapshot["scan"])
            
        if "odom" in snapshot and snapshot["odom"]:
            self.pub_odom.publish(snapshot["odom"])
            
        if "plan" in snapshot and snapshot["plan"]:
            self.pub_plan.publish(snapshot["plan"])
            
        if "goal" in snapshot and snapshot["goal"]:
            self.pub_goal.publish(snapshot["goal"])
            
        if "map" in snapshot and snapshot["map"]:
            self.pub_map.publish(snapshot["map"])
            
        if "gc" in snapshot and snapshot["gc"]:
            self.pub_gc.publish(snapshot["gc"])
            
        if "lc" in snapshot and snapshot["lc"]:
            self.pub_lc.publish(snapshot["lc"])
            
        # Robot Description (Latched)
        if "robot_description" in snapshot:
            desc = snapshot["robot_description"]
            if desc and desc != self._last_robot_desc:
                msg = StringMsg()
                msg.data = desc
                self.pub_robot_desc.publish(msg)
                self._last_robot_desc = desc
                # self.get_logger().info("Published robot_description")
        elif self._last_robot_desc is None:
             # Try to get it from core if not in snapshot (e.g. if snapshot is sparse)
             # But snapshot_service should have put it there if available.
             pass

        if "tf_msg" in snapshot and snapshot["tf_msg"]:
            self.pub_tf.publish(snapshot["tf_msg"])
            
        if "tf_static_msg" in snapshot and snapshot["tf_static_msg"]:
            # Publish static TFs. 
            # We can publish them every time or check if we already did.
            # Since they are static and latched, publishing once is enough, but republishing doesn't hurt much (except bandwidth).
            # Given we are scrubbing, republishing ensures they are available if RViz was restarted.
            self.pub_tf_static.publish(snapshot["tf_static_msg"])



# ======================================================================
# NODE STARTUP HELPERS
# ======================================================================

def start_ros_in_thread(shared: SharedState):
    global _NODE_SINGLETON
    try:
        rclpy.init()
    except RuntimeError as e:
        if "already been called" not in str(e):
            raise

    node = CmdVelNode(shared)
    _NODE_SINGLETON = node

    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    node.get_logger().info("[cmdvel_dash] ROS thread started.")

    def spin():
        try:
            ex.spin()
        finally:
            ex.shutdown()
            node.get_logger().info("[cmdvel_dash] Executor shutdown.")
            ex.remove_node(node)
            node.destroy_node()

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    return th


def request_reverse_replay(speed: float = None) -> bool:
    return _NODE_SINGLETON.start_reverse_replay(speed) if _NODE_SINGLETON else False


def pause_ros():
    if _NODE_SINGLETON:
        _NODE_SINGLETON.pause()


def resume_ros():
    if _NODE_SINGLETON:
        _NODE_SINGLETON.resume()


def publish_bag_snapshot(t: float, snapshot: Dict[str, Any]):
    if _NODE_SINGLETON:
        _NODE_SINGLETON.publish_snapshot(t, snapshot)

