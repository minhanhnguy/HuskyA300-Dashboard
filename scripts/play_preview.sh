#!/bin/bash
# Play back the FIXED preview_of_map bag with correct remappings and QoS

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# Check if QoS file exists
QOS_FILE="$ROOT_DIR/playback_qos.yaml"
if [ ! -f "$QOS_FILE" ]; then
    echo "Creating QoS overrides file..."
    cat > "$QOS_FILE" << EOF
/a300_0000/map:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
/map:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
/a300_0000/tf_static:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
/tf_static:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
/a300_0000/robot_description:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
/robot_description:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
/a300_0000/downsampled_costmap:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
/downsampled_costmap:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
EOF
fi

echo "Playing FIXED bag with remappings and QoS overrides..."
echo "Bag: preview_of_map_fixed.mcap"
echo "Remapping: /a300_0000/tf -> /tf"
echo "Remapping: /a300_0000/tf_static -> /tf_static"
echo "Remapping: /a300_0000/robot_description -> /robot_description"
echo "Remapping: /a300_0000/map -> /map"

ros2 bag play "$ROOT_DIR/public/bag_files/preview_of_map_fixed.mcap" \
    --qos-profile-overrides-path "$QOS_FILE" \
    --remap \
    /a300_0000/tf:=/tf \
    /a300_0000/tf_static:=/tf_static \
    /a300_0000/robot_description:=/robot_description \
    /a300_0000/map:=/map \
    /a300_0000/scan:=/scan \
    /a300_0000/platform/odom:=/odom
