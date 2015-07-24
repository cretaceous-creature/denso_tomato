;;
;; setup-hrp2-tomato.l
;;

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")

(defvar *real* t)
(defvar *base-time* 1500)

(defun setup-hrp2-tomato ()
  (unless (boundp '*hrp2*)
    (hrp2w-init :sim t)
    (setq *hrp2* *hrp2w*)
    (setq ((send *hrp2w* :rarm :thumb-r) . max-angle) 12)
    (setq ((send *hrp2w* :larm :thumb-r) . min-angle) -12)
    )
  (send *hrp2w* :reset-manip-pose)
  (send *hrp2w* :head-neck-p :joint-angle 15)
  (send *hrp2w* :fix-leg-to-coords (make-coords))
  ;;draw hako
  (unless (boundp '*hako*)
    (let (hako-all hako-off)
      ;;(setq hako-all (make-cube 260 380 260)) ;;the origin point is located in the center
      ;;(setq hako-off (make-cube 220 340 160))
      ;;(send hako-off :locate #f(0 0 100))
      (setq hako-all (make-cube 250 350 100)) ;;the origin point is located in the center
      (setq hako-off (make-cube 220 330 70))
      (send hako-off :locate #f(0 0 30))
      (setq *hako* (body- hako-all hako-off))
      ;;(send *hako* :locate #f(240 -20 540))
      (send *hako* :locate #f(230 -20 470))
      (send *hako* :set-color :brown)
      (send *hrp2w* :assoc *hako*)
      ))
  ;;12.13
  (objects (list *hrp2w* *hako*))
  )
(setup-hrp2-tomato)

(defun reset-tomato-pose (&optional (arm :rarm))
  (send *hrp2* :head :angle-vector (float-vector 0.0 40.0))
  (when (or (eq arm :both) (eq arm :larm))
    (warn ";; reset-tomato-pose :larm~%")
    (send *hrp2* :larm :angle-vector
	  (float-vector 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
    )
  (when (or (eq arm :both) (eq arm :rarm))
    (warn ";; reset-tomato-pose :rarm~%")
    (send *hrp2* :rarm :angle-vector
	  (float-vector -5.51515 -22.921 29.4258 -100.919 -18.4809 7.45084 5.12168 12.0)))
  (when (boundp '*ri*)
    (send *ri* :stop-grasp :larm)
    (send *ri* :angle-vector (send *hrp2* :angle-vector) 5000)
    (send *ri* :wait-interpolation))
  )

(load "tomato-motion-lib-custom-chen.l")
