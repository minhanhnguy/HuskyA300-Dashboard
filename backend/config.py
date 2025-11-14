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
MAP_WS_HZ          = 15.0
MAP_TILE_SIZE      = 256

# ---- Pose source (live) ----
# Force odom/filtered only in live mode (no TF, no integration).
POSE_PREF_ORDER = ["odom"]
ODOM_CANDIDATES = [f"{NAMESPACE}/platform/odom/filtered"]
USE_TF_POSE = False  # live

MAP_FRAME   = "map"
BASE_FRAME  = f"{NAMESPACE}/base_link"

# ---- Bag-mode options ----
# If TF exists in the bag, we’ll compose map->odom(t) with odom pose.
# If TF is missing, we can optionally anchor first odom pose to map origin (translate+rotate).
BAG_USE_TF = True
BAG_ANCHOR_ODOM_TO_MAP = True
BAG_USE_MAP_YAW = True  # set False if your frontend does not rotate tiles
