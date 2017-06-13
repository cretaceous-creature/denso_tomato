#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::roseus "eusscissors")
(ros::roseus-add-msgs "trajectory_msgs")
(ros::advertise "/scissors_trajectory_controller/command"
                trajectory_msgs::JointTrajectory 1)

(setq *traj-point*
      (instance trajectory_msgs::JointTrajectoryPoint :init))
(send *traj-point* :positions (float-vector 0.0))
(send *traj-point* :time_from_start (ros::time 1.0))

(setq *traj* (instance trajectory_msgs::JointTrajectory :init))
(send *traj* :joint_names (list "scissors_finger"))
(send *traj* :points (list *traj-point*))

(defun command-finger (angle duration)
  (send *traj-point* :positions (float-vector angle))
  (send *traj-point* :time_from_start (ros::time duration))
  (send *traj* :points (list *traj-point*))
  (ros::publish "/scissors_trajectory_controller/command" *traj-point*)
  )
