# backend/ros_worker.py
"""
ROS-side adapter for the Husky dashboard.

PURPOSE OF THIS FILE (high level):
- Talk to ROS2 (subscribe to /cmd_vel, /odom, /tf, /map, ...).
- Convert ROS messages into plain Python values.
- Hand those values to the *research core* (shared.core), which is the place we
  actually store experiment data for later analysis / for the paper.
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

from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from .state import SharedState
from .utils import clip, wrap_pi
from . import config as C

# we keep a process-global pointer so /api/v1/reverse_replay can trigger it
_NODE_SINGLETON: Optional["CmdVelNode"] = None


class CmdVelNode(Node):
    """
    One ROS2 node that:
    - subscribes to IN and OUT cmd_vel (as TwistStamped),
    - subscribes to map and map patches,
    - tries to discover odometry and/or TF to get the real pose,
    - runs a pose *integrator* as fallback,
    - periodically prints heartbeat,
    - can publish reverse-replay cmd_vel.

    It does NOT keep long histories itself — it passes everything to shared.core.
    """

    def __init__(self, shared: SharedState):
        super().__init__("cmdvel_dash")

        # shared contains the research core (ExperimentCapture)
        self.shared = shared
        self.core = shared.core  # shorter alias

        # which source is currently giving us pose
        # "tf" > "odom" > "integrated"
        self.pose_mode = "integrated"

        # flags to know what we managed to latch on
        self._tf_ok = False
        self._odom_ok = False
        self._odom_topic: Optional[str] = None

        # time for pose-history sampling
        self._last_pose_sample_t = 0.0
        # time for integrator sampling
        self._last_int_pose_sample_t = 0.0

        # remember whether we ever saw a cmd_vel
        self._first_rx = False

        # to smooth v, w when integrating
        self._v_f = 0.0
        self._w_f = 0.0

        # try to run under simulation time (gazebo, rosbag2, etc.)
        from rclpy.parameter import Parameter

        try:
            self.set_parameters([Parameter(name="use_sim_time", value=True)])
        except Exception:
            # if sim time is not available it's fine
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
        # TIMERS
        # ------------------------------------------------------------------
        # integrator: runs at INT_HZ, only used when we don’t have tf/odom
        self.timer = self.create_timer(1.0 / C.INT_HZ, self.integrate_step)
        # heartbeat: 1 Hz print
        self.hb = self.create_timer(1.0, self._heartbeat)

        # ------------------------------------------------------------------
        # TF & ODOM helpers
        # ------------------------------------------------------------------
        # we keep a TF buffer so we can try to get map->base_link and use that pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # poll TF at UI_HZ
        self.tf_timer = self.create_timer(1.0 / C.UI_HZ, self._poll_tf_pose)

        # periodically try to discover an odom topic
        self.odom_probe_timer = self.create_timer(0.5, self._probe_odom_once)

    # ======================================================================
    # CMD_VEL HANDLERS
    # ======================================================================

    def on_cmd_in(self, msg: TwistStamped):
        """
        Called whenever we receive the upstream cmd_vel (the one we want to log).
        We clip it and push it to the research core.
        """
        now_s = self.get_clock().now().nanoseconds * 1e-9
        v = clip(msg.twist.linear.x, -C.V_MAX, C.V_MAX)
        w = clip(msg.twist.angular.z, -C.W_MAX, C.W_MAX)

        # store in research core
        self.core.record_cmd_in(v, w, now_s)

        if not self._first_rx:
            self._first_rx = True
            self.get_logger().info(
                f"[cmdvel_dash] First IN rx: v={v:.3f}, w={w:.3f}"
            )

    def on_cmd_out(self, msg: TwistStamped):
        """
        Called whenever we receive the executed/forwarded cmd_vel.
        We also push it to the research core so we can compare later.
        """
        now_s = self.get_clock().now().nanoseconds * 1e-9
        v = clip(msg.twist.linear.x, -C.V_MAX, C.V_MAX)
        w = clip(msg.twist.angular.z, -C.W_MAX, C.W_MAX)

        self.core.record_cmd_out(v, w, now_s)

    # ======================================================================
    # MAP HANDLERS
    # ======================================================================

    def on_map_full(self, msg: OccupancyGrid):
        """
        Called when SLAM publishes a full OccupancyGrid.
        We convert ROS msg -> numpy array and hand it to the core.
        """
        info = msg.info
        w, h = info.width, info.height

        # make a HxW int8 array
        arr = (
            np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
        )

        # compute yaw of map origin from quaternion
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

        # store in core (core will bump version and clear patch queue)
        self.core.set_full_map(arr, meta, C.MAP_TILE_SIZE)
        self.get_logger().info(
            f"[map] full map v{self.core.map_version}: {w}x{h} @ {info.resolution} m"
        )

    def on_map_patch(self, up: OccupancyGridUpdate):
        """
        Called when SLAM publishes a partial update.
        We forward exact patch indices to the core so the frontend
        can redraw only those tiles.
        """
        patch = {
            "x": int(up.x),
            "y": int(up.y),
            "w": int(up.width),
            "h": int(up.height),
            "data": list(up.data),
        }
        self.core.apply_map_patch(patch)

    # ======================================================================
    # ODOM / TF DISCOVERY AND POSE UPDATES
    # ======================================================================

    def _probe_odom_once(self):
        """
        Called every 0.5s until we find an Odometry topic.
        Tries preferred candidates from config first, then any nav_msgs/Odometry.
        """
        if self._odom_topic is not None:
            return
        if "odom" not in C.POSE_PREF_ORDER:
            return

        names_types = dict(self.get_topic_names_and_types())
        odom_found = None

        # try config-defined candidates first
        for topic in getattr(C, "ODOM_CANDIDATES", []):
            if topic in names_types and any(
                t.endswith("nav_msgs/msg/Odometry") for t in names_types[topic]
            ):
                odom_found = topic
                break

        # otherwise pick first odom we see
        if odom_found is None:
            for name, types in names_types.items():
                if any(t.endswith("nav_msgs/msg/Odometry") for t in types):
                    odom_found = name
                    break

        if odom_found:
            self.create_subscription(Odometry, odom_found, self.on_odom, 10)
            self._odom_topic = odom_found
            self.get_logger().info(f"[odom] Subscribed to {odom_found}")

    def on_odom(self, msg: Odometry):
        """
        If we have odom, we can use it as a higher-quality pose source than the integrator.
        We still push the pose to the research core so the UI can time-scrub it.
        """
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)

        # get yaw from quaternion
        q = msg.pose.pose.orientation
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        yaw = math.atan2(t3, t4)

        t_now = self.get_clock().now().nanoseconds * 1e-9

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
        # prefer odom if TF is not yet ready
        if not self._tf_ok and "odom" in C.POSE_PREF_ORDER:
            self.pose_mode = "odom"

    def _resolve_tf_frames_once(self) -> bool:
        """
        Try to find a valid (map-like) frame and a base_link-like frame once.
        We try a few candidates based on config + namespace.
        """
        if getattr(self, "_tf_map_frame", None) and getattr(
            self, "_tf_base_frame", None
        ):
            return True

        map_candidates = [
            getattr(C, "MAP_FRAME", "map"),
            f"{C.NAMESPACE}/map",
            "map",
        ]
        base_candidates = [
            getattr(C, "BASE_FRAME", f"{C.NAMESPACE}/base_link"),
            f"{C.NAMESPACE}/base_link",
            "base_link",
            f"{C.NAMESPACE}/base_footprint",
            "base_footprint",
        ]

        for m in map_candidates:
            for b in base_candidates:
                try:
                    _ = self.tf_buffer.lookup_transform(
                        m, b, Time(), timeout=Duration(seconds=0.2)
                    )
                    # if we get here, this pair works
                    self._tf_map_frame = m
                    self._tf_base_frame = b
                    self.get_logger().info(f"[tf] Using frames: {m} -> {b}")
                    return True
                except Exception:
                    continue

        return False

    def _poll_tf_pose(self):
        """
        Called at UI_HZ to try to get pose from TF.
        If this succeeds, TF becomes the preferred pose source.
        """
        if "tf" not in C.POSE_PREF_ORDER:
            return
        if not getattr(C, "USE_TF_POSE", True):
            return
        if not self._resolve_tf_frames_once():
            self._tf_ok = False
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self._tf_map_frame,
                self._tf_base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)

            q = tf.transform.rotation
            ysqr = q.y * q.y
            t3 = 2.0 * (q.w * q.z + q.x * q.y)
            t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
            yaw = math.atan2(t3, t4)

            t_now = self.get_clock().now().nanoseconds * 1e-9

            with self.core.lock:
                self.core.x = tx
                self.core.y = ty
                self.core.yaw = yaw
                if (
                    not self.core.path
                    or (tx - self.core.path[-1][0]) ** 2
                    + (ty - self.core.path[-1][1]) ** 2
                    >= C.MOVE_EPS ** 2
                ):
                    self.core.path.append((tx, ty))
                if (t_now - self._last_pose_sample_t) >= (1.0 / C.UI_HZ):
                    self.core.pose_history.append((t_now, tx, ty, yaw))
                    self._last_pose_sample_t = t_now

            self._tf_ok = True
            self.pose_mode = "tf"
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._tf_ok = False

    # ======================================================================
    # INTEGRATOR (dead-reckoning fallback)
    # ======================================================================

    def _smooth(self, v: float, w: float):
        """
        Optional low-pass so the integrated path is less jittery.
        """
        if not C.SMOOTHING:
            return v, w
        self._v_f = C.ALPHA * self._v_f + (1.0 - C.ALPHA) * v
        self._w_f = C.ALPHA * self._w_f + (1.0 - C.ALPHA) * w
        return self._v_f, self._w_f

    def integrate_step(self):
        """
        If we don’t have TF or ODOM, we integrate the last known cmd_vel to get pose.
        This is the same logic you had before, just writing into core.
        """
        # if we have a better source, do nothing
        if self.pose_mode in ("tf", "odom"):
            return

        t_now = self.get_clock().now().nanoseconds * 1e-9

        # how much time since last integration (clamped)
        dt = 1.0 / C.INT_HZ

        # pull current v,w from core (or zero them if they are too old)
        with self.core.lock:
            age = t_now - self.core.last_cmd_time
            if age <= C.CMD_TIMEOUT:
                v = self.core.last_v
                w = self.core.last_w
            else:
                v = 0.0
                w = 0.0

            x = self.core.x
            y = self.core.y
            yaw = self.core.yaw

        v, w = self._smooth(v, w)

        # integrate
        if C.INTEGRATOR.lower() == "rk2":
            yaw_mid = yaw + 0.5 * w * dt
            x += v * math.cos(yaw_mid) * dt
            y += v * math.sin(yaw_mid) * dt
        else:
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
        yaw = wrap_pi(yaw + w * dt)

        # write back
        with self.core.lock:
            self.core.x = x
            self.core.y = y
            self.core.yaw = yaw
            self.core.last_dt_ms = dt * 1000.0
            if (
                not self.core.path
                or (x - self.core.path[-1][0]) ** 2
                + (y - self.core.path[-1][1]) ** 2
                >= C.MOVE_EPS ** 2
            ):
                self.core.path.append((x, y))
            if (t_now - self._last_int_pose_sample_t) >= (1.0 / C.UI_HZ):
                self.core.pose_history.append((t_now, x, y, yaw))
                self._last_int_pose_sample_t = t_now

    # ======================================================================
    # HEARTBEAT
    # ======================================================================

    def _heartbeat(self):
        """
        Just print some counts so we know the node is alive and receiving data.
        """
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
        """
        Publish cmd_vels back in reverse order so we can “retrace” the motion.
        This reads from core.history_{in,out} and publishes to C.PUBLISH_TOPIC.
        """
        from . import config as C2

        if speed is None:
            speed = C2.REPLAY_SPEED

        # decide which stream to use
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

        # compute time gaps
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
                    # stamp mode: now or zero, same as original
                    if C2.PUBLISH_STAMP_MODE == "now":
                        msg.header.stamp = self.get_clock().now().to_msg()
                    else:
                        msg.header.stamp = RosTime(sec=0, nanosec=0)
                    msg.header.frame_id = C2.PUBLISH_FRAME_ID
                    # reverse motion: invert linear, keep angular or invert? original inverted both
                    msg.twist.linear.x = -v
                    msg.twist.angular.z = -w
                    self.pub.publish(msg)
                    if i > 0:
                        time.sleep(dts[i - 1] / max(1e-6, speed))
            finally:
                # turn recording back on
                with self.core.lock:
                    self.core.replaying = False
                    self.core.recording = True
                self.get_logger().info("Reverse replay finished.")

        threading.Thread(target=_thread, daemon=True).start()
        return True


# ======================================================================
# NODE STARTUP HELPERS
# ======================================================================


def start_ros_in_thread(shared: SharedState):
    """
    Called by FastAPI on startup.
    Creates the node, adds it to a SingleThreadedExecutor, spins it in a thread.
    """
    global _NODE_SINGLETON
    try:
        rclpy.init()
    except RuntimeError as e:
        # rclpy.init() can only be called once per process
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
    """
    FastAPI uses this to trigger reverse replay via HTTP.
    """
    return _NODE_SINGLETON.start_reverse_replay(speed) if _NODE_SINGLETON else False
