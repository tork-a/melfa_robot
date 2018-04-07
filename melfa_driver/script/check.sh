#!/bin/sh

rostopic pub /position_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'joint1'
- 'joint2'
- 'joint3'
- 'joint4'
- 'joint5'
- 'joint6'
points:
- positions: [0, 0, 0, 0, 0, 0]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 3, nsecs: 0}
- positions: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 6, nsecs: 0}
- positions: [0, 0, 0, 0, 0, 0]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 9, nsecs: 0}
- positions: [-1.5, 1.5, 0, -1.5, -1.5, -1.5]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 12, nsecs: 0}
- positions: [0, 0, 0, 0, 0, 0]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 15, nsecs: 0}
"
