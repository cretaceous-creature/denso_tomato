(load "package://ureus/models/ur5_robot.l")
(load "package://jsk_tomato_robot/euslisp/tomato-hand.l")

(defun jsk-tomato-robot-init ()
  (if (not (boundp '*tomato-hand*))
      (setq *tomato-hand* (make-tomato-hand)))
  (if (not (boundp '*ur5*))
      (setq *ur5* (instance ur5-robot :init)))
  (send *tomato-hand* :newcoords
        (send *ur5* :rarm :end-coords :copy-worldcoords))
  (send *ur5* :rarm :end-coords :assoc *tomato-hand*)
  (list *ur5* *tomato-hand*))