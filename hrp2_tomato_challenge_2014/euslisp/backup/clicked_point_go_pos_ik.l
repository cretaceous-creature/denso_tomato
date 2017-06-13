
#!/usr/bin/env roseus
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
(ros::roseus "go_pos_pt")
(ros::advertise "/go_pos_pt" geometry_msgs::PointStamped 1)

(defun preparation
  ()
  (hrp2w-init)
  (objects *hrp2w*)
  (send *hrp2w* :reset-manip-pose)
  (send *hrp2w* :fix-leg-to-coords (make-coords))
  (send *hrp2w* :head-neck-p :joint-angle 15)
  (send *irtviewer* :draw-objects)
  )

(defun move-robot
  ()
  (format t "WARN: HRP2-w moves!!!!~%")
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun 2d-nav-goal-cb (msg)
  (setq *msg* msg)
  (setq *torso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))
  (setq *nav-goal* (make-coords))
  (send *nav-goal* :locate (float-vector (* 1000 (send *msg* :pose :position :x))
                                         (* 1000 (send *msg* :pose :position :y))
                                         (* 1000 (send *msg* :pose :position :z))))
  ;; set rotation from msg to nav-goal here


  (format t "~A~%" *nav-goal*)
  (setq *nav-pt* (send *torso-co* :transform *nav-goal*))
  (format t "~A~%" *nav-pt*)

  ;; go-pos for *vmax*
  (setq x (- (elt (send *nav-pt* :pos) 0) 300))
  (setq y (elt (send *nav-pt* :pos) 1))
  (setq dt (* 10 (norm (send *nav-pt* :pos)))) ;; duration depending on distance

  ;; PointStamped for debugging
  (setq header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "CHEST_LINK0"))
  (setq pt (instance geometry_msgs::Point :init :x (* 0.001 x) :y (* 0.001 y) :z 0))
  (setq *point-stamped* (instance geometry_msgs::PointStamped :init :header header :point pt))
  (ros::publish "/go_pos_pt" *point-stamped*)

  (format t "Press ENTER to go pos at x=~Am y=~Am v=~Asec~%"
          (send *point-stamped* :point :x)
          (send *point-stamped* :point :y)
          (* 0.001 dt))
  (read-line)

  (send *vmax* :go-pos-relative-unsafe x y 0 dt)
  (send *vmax* :wait-interpolation) ;; for safety
  )

(defun clicked-pt-cb (msg)
  (setq *msg* msg)
  (setq *torso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))
  (setq *clicked-pt* (make-coords))
  (send *clicked-pt* :locate (float-vector (* 1000 (send *msg* :point :x))
                                           (* 1000 (send *msg* :point :y))
                                           (* 1000 (send *msg* :point :z))))
  (setq *ik-target* (send *torso-co* :transform *clicked-pt*))
  (setq *rarm-end* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
  (send *rarm-end* :locate (send *ik-target* :pos) :world)
  (send *rarm-end* :translate #f(0 0 0))

  ;; need conditional branch if IK fails
  (send *hrp2w* :rarm :inverse-kinematics *rarm-end*
        :rotation-axis nil
        :debug-view t :torso nil
        :link-list (send *hrp2w* :link-list
                         (send *hrp2w* :rarm :end-coords :parent)
                         (car (send *hrp2w* :torso :links))))
  (send *hrp2w* :wait-interpolation) ;; for safety
  )

(setq *msg* nil)

(preparation)
(format t "Press y to reset-manip-pose!~%")
(setq ans (read-line)) ;; for safety
(if (or (equal ans "y") (equal ans "Y"))
    (move-robot))

(ros::roseus "clicked_point_go_pos_ik")
(ros::subscribe "clicked_point" geometry_msgs::PointStamped #'clicked-pt-cb)
(ros::subscribe "move_base_simple/goal" geometry_msgs::PoseStamped #'2d-nav-goal-cb)

(do-until-key
 (ros::spin-once)
 )