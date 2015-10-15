#!/bin/bash

rostopic pub /dummy_state pr2_controllers_msgs/JointTrajectoryControllerState \
"header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['']
desired:
  positions: [0]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}
actual:
  positions: [0]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}
error:
  positions: [0]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}"
