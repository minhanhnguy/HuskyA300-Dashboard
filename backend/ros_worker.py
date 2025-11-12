# backend/ros_worker.py
import math, threading, time
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from .state import SharedState
from .utils import clip, wrap_pi
from . import config as C

_NODE_SINGLETON = None


class CmdVelNode(Node):
    def __init__(self, shared: SharedState):
        super().__init__('cmdvel_dash')
        self.shared = shared

        # Pose source state
        self.pose_mode = "integrated"   # "tf" | "odom" | "integrated"
        self._tf_ok = False
        self._odom_ok = False
        self._odom_topic = None
        self.odom_sub = None

        self._last_pose_sample_t = 0.0
        self._first_rx = False

        # sim time
        from rclpy.parameter import Parameter
        try:
            self.set_parameters([Parameter(name='use_sim_time', value=True)])
        except Exception:
            pass

        # shared buffers
        with self.shared.lock:
            self.shared.path = deque(maxlen=C.PATH_MAX_POINTS)
            self.shared.history_in = deque(maxlen=C.HISTORY_MAX)
            self.shared.history_out = deque(maxlen=C.HISTORY_MAX)
            self.shared.pose_history = deque(maxlen=C.POSE_HISTORY_MAX)
            if C.SEED_MODE == "manual":
                self.shared.seed(C.SEED_X, C.SEED_Y, C.SEED_YAW)
            else:
                self.shared.seed(0.0, 0.0, 0.0)

        # ---- Subscriptions ----
        # IN (upstream cmd_vel) — TwistStamped (your topic type)
        self.sub_in = self.create_subscription(
            TwistStamped, C.RECORD_IN_TOPIC, self.on_cmd_in, 50
        )
        # OUT (executed cmd_vel) if present
        self.sub_out = self.create_subscription(
            TwistStamped, C.RECORD_OUT_TOPIC, self.on_cmd_out, 50
        )
        # SLAM map
        self.sub_map = self.create_subscription(
            OccupancyGrid, C.MAP_TOPIC, self.on_map_full, 1
        )
        self.sub_map_up = self.create_subscription(
            OccupancyGridUpdate, C.MAP_UPDATES_TOPIC, self.on_map_patch, 50
        )

        self.get_logger().info(f"[cmdvel_dash] Record IN {C.RECORD_IN_TOPIC}, OUT {C.RECORD_OUT_TOPIC}")
        self.get_logger().info(f"[map] Subscribed to {C.MAP_TOPIC} and {C.MAP_UPDATES_TOPIC}")

        # ---- Publisher (reverse replay) ----
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, C.PUBLISH_TOPIC, qos)
        self.get_logger().info(f"[cmdvel_dash] Replay publisher -> {C.PUBLISH_TOPIC}")

        # ---- Timers ----
        self.timer = self.create_timer(1.0 / C.INT_HZ, self.integrate_step)
        self.hb = self.create_timer(1.0, self._heartbeat)

        # TF & ODOM discovery
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(1.0 / C.UI_HZ, self._poll_tf_pose)
        self._tf_map_frame = None
        self._tf_base_frame = None

        # Try to latch onto an odometry topic automatically
        self.odom_probe_timer = self.create_timer(0.5, self._probe_odom_once)

        # smoothing
        self._v_f = 0.0
        self._w_f = 0.0

    # ---------- CMD_VEL inbound ----------
    def on_cmd_in(self, msg: TwistStamped):
        self._handle_cmd(msg, record="in")

    def on_cmd_out(self, msg: TwistStamped):
        self._handle_cmd(msg, record="out")

    def _handle_cmd(self, msg: TwistStamped, record: str):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        v = clip(msg.twist.linear.x, -C.V_MAX, C.V_MAX)
        w = clip(msg.twist.angular.z, -C.W_MAX, C.W_MAX)

        if record == "in":
            with self.shared.lock:
                self.shared.v = v
                self.shared.w = w
                self.shared.stats.last_cmd_time = now_s
                self.shared.stats.rx_count += 1
                self.shared.stats.last_v = v
                self.shared.stats.last_w = w
                if self.shared.recording and not self.shared.replaying:
                    self.shared.history_in.append((v, w, now_s))
            if not self._first_rx:
                self._first_rx = True
                self.get_logger().info(f"[cmdvel_dash] First IN rx: v={v:.3f}, w={w:.3f}")
        else:
            with self.shared.lock:
                if self.shared.recording and not self.shared.replaying:
                    self.shared.history_out.append((v, w, now_s))

    # ---------- Map handlers ----------
    def on_map_full(self, msg: OccupancyGrid):
        info = msg.info
        w, h = info.width, info.height
        arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)

        # yaw from quaternion (x,y,z,w)
        q = info.origin.orientation
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        yaw0 = math.atan2(t3, t4)

        with self.shared.lock:
            self.shared.map_grid = arr  # [H,W] int8
            self.shared.map_version += 1
            self.shared.map_info = {
                "width": int(w),
                "height": int(h),
                "resolution": float(info.resolution),
                "origin": {
                    "x": float(info.origin.position.x),
                    "y": float(info.origin.position.y),
                    "yaw": float(yaw0),
                },
                "version": int(self.shared.map_version),
                "tile_size": int(C.MAP_TILE_SIZE),
            }
            self.shared.map_patch_queue.clear()
        self.get_logger().info(f"[map] full map v{self.shared.map_version}: {w}x{h} @ {info.resolution} m")

    def on_map_patch(self, up: OccupancyGridUpdate):
        with self.shared.lock:
            if self.shared.map_grid is None:
                return
            g = self.shared.map_grid
            w, h = up.width, up.height
            x0, y0 = up.x, up.y

        patch = np.asarray(up.data, dtype=np.int8).reshape(h, w)

        with self.shared.lock:
            g[y0:y0 + h, x0:x0 + w] = patch
            self.shared.map_patch_queue.append({
                "x": int(x0), "y": int(y0), "w": int(w), "h": int(h),
                "data": patch.flatten(order='C').tolist()
            })

    # ---------- Pose: TF / ODOM / INTEGRATED ----------
    def _probe_odom_once(self):
        if self.odom_sub is not None:
            return
        if "odom" not in C.POSE_PREF_ORDER:
            return

        names_types = dict(self.get_topic_names_and_types())
        odom_found = None
        # Preferred candidates first
        for topic in getattr(C, "ODOM_CANDIDATES", []):
            if topic in names_types and any(t.endswith("nav_msgs/msg/Odometry") for t in names_types[topic]):
                odom_found = topic
                break
        # Otherwise pick the first Odometry topic present
        if odom_found is None:
            for name, types in names_types.items():
                if any(t.endswith("nav_msgs/msg/Odometry") for t in types):
                    odom_found = name
                    break

        if odom_found:
            self.odom_sub = self.create_subscription(Odometry, odom_found, self.on_odom, 10)
            self._odom_topic = odom_found
            self.get_logger().info(f"[odom] Subscribed to {odom_found}")

    def on_odom(self, msg: Odometry):
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        ysqr = q.y * q.y
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
        yaw = math.atan2(t3, t4)

        t_now = self.get_clock().now().nanoseconds * 1e-9
        with self.shared.lock:
            self.shared.x, self.shared.y, self.shared.yaw = px, py, yaw
            if (not self.shared.path) or ((px - self.shared.path[-1][0]) ** 2 + (py - self.shared.path[-1][1]) ** 2) >= C.MOVE_EPS ** 2:
                self.shared.path.append((px, py))
            if (t_now - self._last_pose_sample_t) >= (1.0 / C.UI_HZ):
                self.shared.pose_history.append((t_now, px, py, yaw))
                self._last_pose_sample_t = t_now
        self._odom_ok = True
        if not self._tf_ok and "odom" in C.POSE_PREF_ORDER:
            self.pose_mode = "odom"

    def _resolve_tf_frames_once(self):
        if self._tf_map_frame and self._tf_base_frame:
            return True

        map_candidates = [getattr(C, "MAP_FRAME", "map"), f"{C.NAMESPACE}/map", "map"]
        base_candidates = [
            getattr(C, "BASE_FRAME", f"{C.NAMESPACE}/base_link"),
            f"{C.NAMESPACE}/base_link", "base_link",
            f"{C.NAMESPACE}/base_footprint", "base_footprint",
        ]
        for m in map_candidates:
            for b in base_candidates:
                try:
                    _ = self.tf_buffer.lookup_transform(m, b, Time(), timeout=Duration(seconds=0.2))
                    self._tf_map_frame, self._tf_base_frame = m, b
                    self.get_logger().info(f"[tf] Using frames: {m} -> {b}")
                    return True
                except Exception:
                    continue
        return False

    def _poll_tf_pose(self):
        if "tf" not in C.POSE_PREF_ORDER:
            return
        if not getattr(C, "USE_TF_POSE", True):
            return
        if not self._resolve_tf_frames_once():
            self._tf_ok = False
            return

        try:
            tf = self.tf_buffer.lookup_transform(self._tf_map_frame, self._tf_base_frame, Time(), timeout=Duration(seconds=0.2))
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            q = tf.transform.rotation
            ysqr = q.y * q.y
            t3 = 2.0 * (q.w * q.z + q.x * q.y)
            t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
            yaw = math.atan2(t3, t4)

            t_now = self.get_clock().now().nanoseconds * 1e-9
            with self.shared.lock:
                self.shared.x, self.shared.y, self.shared.yaw = tx, ty, yaw
                if (not self.shared.path) or ((tx - self.shared.path[-1][0]) ** 2 + (ty - self.shared.path[-1][1]) ** 2) >= C.MOVE_EPS ** 2:
                    self.shared.path.append((tx, ty))
                if (t_now - self._last_pose_sample_t) >= (1.0 / C.UI_HZ):
                    self.shared.pose_history.append((t_now, tx, ty, yaw))
                    self._last_pose_sample_t = t_now
            self._tf_ok = True
            self.pose_mode = "tf"
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._tf_ok = False

    # ---------- Integrator (dead-reckoning) ----------
    def _smooth(self, v, w):
        if not C.SMOOTHING:
            return v, w
        self._v_f = C.ALPHA * self._v_f + (1.0 - C.ALPHA) * v
        self._w_f = C.ALPHA * self._w_f + (1.0 - C.ALPHA) * w
        return self._v_f, self._w_f

    def integrate_step(self):
        # Skip integration only if a higher-priority pose source is active
        if self.pose_mode in ("tf", "odom"):
            return

        t_now = self.get_clock().now().nanoseconds * 1e-9
        with self.shared.lock:
            last_t = self.shared.last_update_time or (t_now - 1.0 / C.INT_HZ)
        dt = max(C.DT_MIN, min(t_now - last_t, C.DT_MAX))

        with self.shared.lock:
            self.shared.last_update_time = t_now
            age = t_now - self.shared.stats.last_cmd_time
            v, w = (self.shared.v, self.shared.w) if age <= C.CMD_TIMEOUT else (0.0, 0.0)
            v, w = self._smooth(v, w)

            x, y, yaw = self.shared.x, self.shared.y, self.shared.yaw
            if C.INTEGRATOR.lower() == "rk2":
                yaw_mid = yaw + 0.5 * w * dt
                x += v * math.cos(yaw_mid) * dt
                y += v * math.sin(yaw_mid) * dt
            else:
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
            yaw = wrap_pi(yaw + w * dt)

            if (not self.shared.path) or ((x - self.shared.path[-1][0]) ** 2 + (y - self.shared.path[-1][1]) ** 2) >= C.MOVE_EPS ** 2:
                self.shared.path.append((x, y))

            self.shared.x, self.shared.y, self.shared.yaw = x, y, yaw
            self.shared.stats.last_dt_ms = dt * 1000.0

            # Pose timeline (~UI_HZ)
            if (t_now - self._last_pose_sample_t) >= (1.0 / C.UI_HZ):
                self.shared.pose_history.append((t_now, x, y, yaw))
                self._last_pose_sample_t = t_now

    # ---------- Heartbeat ----------
    def _heartbeat(self):
        with self.shared.lock:
            st = self.shared.stats
            npts = len(self.shared.path)
            hin = len(self.shared.history_in)
            hout = len(self.shared.history_out)
            ph = len(self.shared.pose_history)
            mver = self.shared.map_version
            mgrid = self.shared.map_grid
            msz = None if mgrid is None else (mgrid.shape[1], mgrid.shape[0])
        self.get_logger().info(
            f"[hb] v={st.last_v:+.2f}, w={st.last_w:+.2f}, dt={st.last_dt_ms:6.3f} ms | "
            f"path={npts}, hist_in={hin}, hist_out={hout}, pose_hist={ph} | map v{mver} size={msz} | "
            f"pose_mode={self.pose_mode}, tf_ok={self._tf_ok}, odom_ok={self._odom_ok}, odom_topic={self._odom_topic}"
        )

    # ---------- Reverse replay ----------
    def start_reverse_replay(self, speed: float = None):
        from . import config as C2
        if speed is None:
            speed = C2.REPLAY_SPEED
        with self.shared.lock:
            if self.shared.replaying:
                self.get_logger().warn("Replay already running")
                return False
            use_out = self.shared.replay_source == "out" and len(self.shared.history_out) > 1
            hist = list(self.shared.history_out) if use_out else list(self.shared.history_in)
            self.shared.replaying = True
            self.shared.recording = False
        if len(hist) < 2:
            self.get_logger().warn("Not enough history to replay")
            with self.shared.lock:
                self.shared.replaying = False
                self.shared.recording = True
            return False

        times = [t for (_, _, t) in hist]
        dts = [max(0.0, times[i + 1] - times[i]) for i in range(len(times) - 1)]

        def _thread():
            self.get_logger().info(
                f"Reverse replay using {'EXECUTED' if hist is self.shared.history_out else 'INPUT'} stream: "
                f"{len(hist)} samples, speed={speed}x"
            )
            try:
                for i in range(len(hist) - 1, -1, -1):
                    v, w, _t = hist[i]
                    msg = TwistStamped()
                    if C2.PUBLISH_STAMP_MODE == 'now':
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
                with self.shared.lock:
                    self.shared.replaying = False
                    self.shared.recording = True
                self.get_logger().info("Reverse replay finished.")

        threading.Thread(target=_thread, daemon=True).start()
        return True


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
