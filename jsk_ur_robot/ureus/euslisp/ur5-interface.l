(load "package://pr2eus/robot-interface.l")
(require :ur5-robot "package://ureus/models/ur5_robot.l")

(defclass ur5-interface
  :super robot-interface
  :slots ())

(defmethod ur5-interface
  (:init
   (&rest args)
   (send-super* :init
                :robot ur5-robot
                :joint-state-topic "/joint_states"
                :type :ur5-controller
                :groupname "ur5_interface"
                args)
   self)
  (:ur5-controller
   ()
   (list
    (list
     (cons :controller-action "/arm_controller/follow_joint_trajectory")
     (cons :controller-state "/arm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
           (mapcar
            #'(lambda (n)
                (if (symbolp n) (symbol-name n) n))
            (send-all (send robot :joint-list) :name)))
     ))
   )
  )

(defun ur5-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* ur5-interface :init args)))
  (if (not (boundp '*ur5*))
      (setq *ur5* (instance ur5-robot :init)))
  )
