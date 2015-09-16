(load "package://jsk_tomato_robot/euslisp/jsk-tomato-robot.l")
(load "package://jsk_tomato_robot/euslisp/tomato-model.l")

(unless (boundp '*ur5*) (jsk-tomato-robot-init))

(unless (boundp '*tomato-list*)
  (let (tomato0 tomato1 tomato2)
    (setq tomato0 (make-tomato))
    (send tomato0 :translate (float-vector 0 600 800))

    (setq tomato1 (make-tomato))
    (send tomato1 :translate (float-vector -40 640 850))
    (send tomato1 :rotate (deg2rad 45) :x)
    (send tomato1 :rotate (deg2rad 15) :y)

    (setq tomato2 (make-tomato))
    (send tomato2 :translate (float-vector 40 640 850))
    (send tomato2 :rotate (deg2rad 45) :x)
    (send tomato2 :rotate (deg2rad -15) :y)

    (setq *tomato-list* (list tomato0 tomato1 tomato2))
    ))

(setq *real* t)
(defun send-angle-vector (&key (time 50))
  (when *real*
    (send *ri* :angle-vector (send *ur5* :angle-vector) time)
    (send *ri* :wait-interpolation)))

(defun reset-ur5-pose ()
  (send *ur5* :reset-pose)
  (send *ur5* :rarm :shoulder-y :joint-angle 90)
  (send *ur5* :rarm :shoulder-p :joint-angle -60)
  (send *ur5* :rarm :elbow-p :joint-angle 120)
  (send *ur5* :rarm :wrist-p :joint-angle -150)
  (send *ur5* :rarm :wrist-y :joint-angle 90))

(reset-ur5-pose)
(objects (flatten (list *ur5* *tomato-hand* *tomato-base* *tomato-list*)))
(send-angle-vector)

;; sample-motion
(defun catch-tomato (&optional (tomato (car *tomato-list*)))
  (when
      (send *ur5* :rarm :inverse-kinematics
            (send tomato :worldcoords)
            :move-target (send *tomato-hand* :target-coords))
    (send *tomato-hand* :catch tomato)
    (send *irtviewer* :draw-objects)
    (send-angle-vector)
    )
  )

(defun put-ur5-pose ()
  (send *ur5* :reset-pose)
  (send *ur5* :rarm :shoulder-y :joint-angle -45)
  (send *ur5* :rarm :shoulder-p :joint-angle -80)
  (send *ur5* :rarm :elbow-p :joint-angle 145)
  (send *ur5* :rarm :wrist-p :joint-angle -155)
  (send *ur5* :rarm :wrist-y :joint-angle 0))

(defun put-tomato ()
  (put-ur5-pose)
  (send *tomato-hand* :release)
  (send *irtviewer* :draw-objects)
  (send-angle-vector))

(defun reset-ur5-pose2 ()
  (reset-ur5-pose)
  (send *irtviewer* :draw-objects)
  (send-angle-vector))
