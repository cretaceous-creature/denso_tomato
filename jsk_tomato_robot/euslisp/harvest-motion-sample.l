(load "package://jsk_tomato_robot/euslisp/jsk-tomato-robot.l")
(load "package://jsk_tomato_robot/euslisp/tomato-model.l")

(unless (boundp '*ur5*)
  (jsk-tomato-robot-init)
  (setq *endco*
        (make-cascoords
         :coords
         (send *ur5* :rarm :end-coords :copy-worldcoords)))
  (send *endco* :translate (float-vector 100 0 0))
  (send *endco* :rotate pi/2 :y)
  (send *ur5* :rarm :end-coords :assoc *endco*)
  )

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


(defun reset-ur5-pose ()
  (send *ur5* :reset-pose)
  (send *ur5* :rarm :shoulder-y :joint-angle 90)
  (send *ur5* :rarm :shoulder-p :joint-angle -60)
  (send *ur5* :rarm :elbow-p :joint-angle 120)
  (send *ur5* :rarm :wrist-p :joint-angle -150)
  (send *ur5* :rarm :wrist-y :joint-angle 90))

(reset-ur5-pose)
(objects (flatten (list *ur5* *tomato-hand* *tomato-base* *tomato-list*)))


;; sample-motion
(defun catch-tomato (&optional (tomato (car *tomato-list*)))
  (when
      (send *ur5* :rarm :inverse-kinematics
            (send tomato :worldcoords)
            :move-target *endco*)
    (send *tomato-hand* :assoc tomato)
    (send *irtviewer* :draw-objects)
    )
  )

(defun put-ur5-pose ()
  (send *ur5* :reset-pose)
  (send *ur5* :rarm :shoulder-y :joint-angle -45)
  (send *ur5* :rarm :shoulder-p :joint-angle -80)
  (send *ur5* :rarm :elbow-p :joint-angle 145)
  (send *ur5* :rarm :wrist-p :joint-angle -155)
  (send *ur5* :rarm :wrist-y :joint-angle 0))

(defun put-tomato (&optional (tomato (car *tomato-list*)))
  (put-ur5-pose)
  (send *tomato-hand* :dissoc tomato)
  (send *irtviewer* :draw-objects))


