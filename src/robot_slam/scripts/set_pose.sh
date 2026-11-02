#!/bin/bash
# 用法: rosrun robot_slam set_pose.sh x y yaw_deg
# 例如: rosrun robot_slam set_pose.sh 0.2 0.2 90

X=${1:-0.2}
Y=${2:-0.2}
YAW=${3:-0}

# 将角度转为四元数
Z=$(echo "s($YAW * 3.14159 / 360)" | bc -l)
W=$(echo "c($YAW * 3.14159 / 360)" | bc -l)

rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: $X, y: $Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $Z, w: $W}
    },
    covariance: [0.25, 0, 0, 0, 0, 0,
                 0, 0.25, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0.0685]
  }
}"

echo "已设置初始位姿: x=$X, y=$Y, yaw=${YAW}°"
