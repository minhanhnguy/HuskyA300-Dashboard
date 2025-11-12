# backend/config.py

NAMESPACE = "/a300_0000"

# ---- Integrator / UI ----
INT_HZ = 150
UI_HZ  = 25
DT_MIN = 1e-4
DT_MAX = 0.2
CMD_TIMEOUT = 0.6
INTEGRATOR = "rk2"
SMOOTHING = False
ALPHA = 0.6
V_MAX = 2.0
W_MAX = 3.0
PATH_MAX_POINTS = 10000
MOVE_EPS = 0.015

# ---- cmd_vel record/publish ----
RECORD_IN_TOPIC  = f"{NAMESPACE}/cmd_vel"                                  # geometry_msgs/TwistStamped (your setup)
RECORD_OUT_TOPIC = f"{NAMESPACE}/platform_velocity_controller/cmd_vel_out" # geometry_msgs/TwistStamped
PUBLISH_TOPIC    = f"{NAMESPACE}/cmd_vel"                                  # where reverse replay publishes
REPLAY_SPEED = 1.0
HISTORY_MAX = 200000
PUBLISH_STAMP_MODE = "now"
PUBLISH_FRAME_ID = ""

POSE_HISTORY_MAX = 100000
SEED_MODE = "zero"
SEED_X = 0.0; SEED_Y = 0.0; SEED_YAW = 0.0

TOPIC_CMD_VEL = RECORD_IN_TOPIC  # back-compat

# ---- Map topics & cadence ----
MAP_TOPIC          = f"{NAMESPACE}/map"
MAP_UPDATES_TOPIC  = f"{NAMESPACE}/map_updates"
MAP_WS_HZ          = 15.0          # push patches at 15 Hz
MAP_TILE_SIZE      = 256           # tile in CELLS (for client-side tiling)

# ---- Pose source preferences & frames ----
# The node will try pose sources in this order:
#   1) TF (map -> base_link)
#   2) ODOM (nav_msgs/Odometry topic)
#   3) INTEGRATED (dead-reckon from cmd_vel)
POSE_PREF_ORDER = ["tf", "odom", "integrated"]

# Add Clearpath A300 platform odom topics explicitly.
ODOM_CANDIDATES = [
    f"{NAMESPACE}/platform/odom/filtered",
    f"{NAMESPACE}/platform/odom",
    f"{NAMESPACE}/odometry/filtered",
    f"{NAMESPACE}/odom",
    f"{NAMESPACE}/wheel/odometry",
    "odometry/filtered",
    "odom",
]

USE_TF_POSE = True
MAP_FRAME   = "map"
BASE_FRAME  = f"{NAMESPACE}/base_link"
