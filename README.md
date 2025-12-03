# HuskyA300-Dashboard

Source the Ros2 Environment with:

```bash
source /opt/ros/jazzy/setup.bash
```

Run the backend with:

```bash
python -m uvicorn backend.app:app --host 0.0.0.0 --port 8000 --reload --workers 1
```

Open another terminal, run the frontend with:

```bash
npm run dev
```

Recording bag file command:

```bash
NS=/a300_0000

ros2 bag record \
  --storage mcap \
  --include-hidden-topics \
  --include-unpublished-topics \
  -o ./husky_bag_$(date +%Y%m%d_%H%M%S)_0.mcap \
  /clock \
  $NS/tf \
  $NS/tf_static \
  $NS/robot_description \
  $NS/platform/joint_states \
  $NS/platform/odom \
  $NS/platform/odom/filtered \
  $NS/pose \
  $NS/particlecloud \
  $NS/initialpose \
  $NS/set_pose \
  $NS/map \
  $NS/map_updates \
  $NS/map_metadata \
  $NS/slam_toolbox/graph_visualization \
  $NS/slam_toolbox/scan_visualization \
  $NS/slam_toolbox/update \
  $NS/global_costmap/costmap \
  $NS/global_costmap/costmap_updates \
  $NS/global_costmap/costmap_raw \
  $NS/global_costmap/costmap_raw_updates \
  $NS/global_costmap/voxel_marked_cloud \
  $NS/local_costmap/costmap \
  $NS/local_costmap/costmap_updates \
  $NS/local_costmap/voxel_marked_cloud \
  $NS/downsampled_costmap \
  $NS/downsampled_costmap_updates \
  $NS/sensors/lidar2d_0/scan \
  $NS/sensors/imu_0/data \
  $NS/plan \
  $NS/plan_smoothed \
  $NS/local_plan \
  $NS/waypoints \
  $NS/route_graph \
  $NS/cmd_vel \
  $NS/platform/cmd_vel \
  $NS/platform_velocity_controller/cmd_vel_out \
  $NS/joy_teleop/cmd_vel \
  $NS/rc_teleop/cmd_vel \
  $NS/twist_marker_server/cmd_vel \
  $NS/platform/emergency_stop \
  $NS/platform/safety_stop \
  $NS/speed_limit \
  $NS/diagnostics \
  $NS/goal_pose
  /parameter_events \
  /rosout
```