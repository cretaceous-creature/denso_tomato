;;
;; tomato-motion-lib.l
;;
(defun reset-place-pose (&optional (arm :rarm))
  (send *hrp2* :head :angle-vector (float-vector 0.0 40.0))
  (when (or (eq arm :both) (eq arm :larm))
    (warn ";; reset-tomato-pose :larm~%")
    (send *hrp2* :larm :angle-vector
	  (float-vector -10.6838 14.2349 -40.7541 -24.6312 -14.7898 -24.0629 -47.3722 12.0))
    )
  (when (or (eq arm :both) (eq arm :rarm))
    (warn ";; reset-tomato-pose :rarm~%")
    (send *hrp2* :rarm :angle-vector
	  (float-vector -8.02614 -18.2593 46.29 -26.3519 16.8251 18.1081 -51.4632 -12.0)))
  (when (boundp '*ri*)
    (send *ri* :angle-vector (send *hrp2* :angle-vector) 5000)
    (send *ri* :wait-interpolation))
    ;;(send *ri* :stop-grasp :larm)
    ;;(send *ri* :stop-grasp :rarm)
    (stop-cut)
    (stop-left-cut))
  
)


(defun start-left-cut ()
;;  (hrp2w-start-grasp :larm :angle 8))
(progn (send *hrp2* :larm :thumb-r :joint-angle 16) (send *ri* :angle-vector (send *hrp2* :angle-vector) :fast :default-controller 0 :min-time 0.2)))

(defun stop-left-cut ()
  (hrp2w-stop-grasp :larm))

(defun start-cut ()
;;  (hrp2w-start-grasp :rarm :angle -8))
(progn (send *hrp2* :rarm :thumb-r :joint-angle -16) (send *ri* :angle-vector (send *hrp2* :angle-vector) :fast :default-controller 0 :min-time 0.2)))


(defun stop-cut ()
  (hrp2w-stop-grasp :rarm))

(defun move-hand-vi
  (&key (real *real*) (robot *hrp2*) (arm :rarm) (move-time (* 0.5 *base-time*))
	(move-dist 20) (move-headp nil))
  (let (key-input velocity (short-dist (/ move-dist 3.0)))
    (when move-headp
      (send *hrp2* :head :look-at (send *hrp2* arm :end-coords :worldpos)))
    (when real
      (send *ri* :angle-vector (send robot :angle-vector) (* 3 *base-time*)))
    (while t
      (setf key-input (read-line))
      (cond
	;;left hand
       ((equal key-input "s") (setf velocity (float-vector 0 0 (- move-dist)))
(setq arm :larm)) ;; "s"=down
       ((equal key-input "w") (setf velocity (float-vector 0 0 move-dist))
(setq arm :larm)) ;; "w"=up
       ((equal key-input "a") (setf velocity (float-vector 0 move-dist 0))
(setq arm :larm)) ;; "a"=left
       ((equal key-input "d") (setf velocity (float-vector 0 (- move-dist) 0))
(setq arm :larm)) ;; "d"=right
       ((equal key-input "r") (setf velocity (float-vector move-dist 0 0))
(setq arm :larm)) ;; "r"=forward
       ((equal key-input "t") (setf velocity (float-vector (- move-dist) 0 0))
(setq arm :larm)) ;; "t"=backward

       ((equal key-input "ss") (setf velocity (float-vector 0 0 (- short-dist)))
(setq arm :larm)) ;; "j"=down
       ((equal key-input "ww") (setf velocity (float-vector 0 0 short-dist))
(setq arm :larm)) ;; "k"=up
       ((equal key-input "aa") (setf velocity (float-vector 0 short-dist 0))
(setq arm :larm)) ;; "h"=left
       ((equal key-input "dd") (setf velocity (float-vector 0 (- short-dist) 0))
(setq arm :larm)) ;; "l"=right
       ((equal key-input "ff") (setf velocity (float-vector short-dist 0 0))
(setq arm :larm)) ;; "f"=forward
       ((equal key-input "rr") (setf velocity (float-vector (- short-dist) 0 0))
(setq arm :larm)) ;; "b"=backward

	;;right hand
       ((equal key-input "k") (setf velocity (float-vector 0 0 (- move-dist)))
(setq arm :rarm)) ;; "s"=down
       ((equal key-input "i") (setf velocity (float-vector 0 0 move-dist))
(setq arm :rarm)) ;; "w"=up
       ((equal key-input "j") (setf velocity (float-vector 0 move-dist 0))
(setq arm :rarm)) ;; "a"=left
       ((equal key-input "l") (setf velocity (float-vector 0 (- move-dist) 0))
(setq arm :rarm)) ;; "d"=right
       ((equal key-input "o") (setf velocity (float-vector move-dist 0 0))
(setq arm :rarm)) ;; "o"=forward
       ((equal key-input "p") (setf velocity (float-vector (- move-dist) 0 0))
(setq arm :rarm)) ;; "p"=backward

       ((equal key-input "kk") (setf velocity (float-vector 0 0 (- short-dist)))
(setq arm :rarm)) ;; "j"=down
       ((equal key-input "ii") (setf velocity (float-vector 0 0 short-dist))
(setq arm :rarm)) ;; "k"=up
       ((equal key-input "jj") (setf velocity (float-vector 0 short-dist 0))
(setq arm :rarm)) ;; "h"=left
       ((equal key-input "ll") (setf velocity (float-vector 0 (- short-dist) 0))
(setq arm :rarm)) ;; "l"=right
       ((equal key-input "oo") (setf velocity (float-vector short-dist 0 0))
(setq arm :rarm)) ;; "f"=forward
       ((equal key-input "pp") (setf velocity (float-vector (- short-dist) 0 0))
(setq arm :rarm)) ;; "b"=backward

       ;;((equal key-input "+") (setf velocity (0 0 0))(start-cut))
       ;;((equal key-input "-") (setf velocity (0 0 0))(stop-cut))



       ((equal key-input "q") (return-from move-hand-vi)) ;; "q"=quit
       )
      (send robot arm :move-end-pos velocity :world :rotation-axis :z)
      (when move-headp
	(send *hrp2* :head :look-at (send *hrp2* arm :end-coords :worldpos)))
      (send *irtviewer* :draw-objects)
      (when real
	(send *ri* :angle-vector (send robot :angle-vector) move-time))
      )
    ))

(defun rot-hand-vi
  (&key (real *real*) (robot *hrp2*) (arm :larm) (move-time (* 0.5 *base-time*))
	(move-ang 10) (move-headp t))
  (let (key-input rot-axis)
    (when move-headp
      (send *hrp2* :head :look-at (send *hrp2* arm :end-coords :worldpos)))
    (when real
      (send *ri* :angle-vector (send robot :angle-vector) (* 3 *base-time*)))
    (while t
      (setf key-input (read-line))
      (cond
       ((equal key-input "j") (setf rot-axis :z)) ;; z
       ((equal key-input "k") (setf rot-axis :-z)) ;; -z
       ((equal key-input "h") (setf rot-axis :x)) ;; x
       ((equal key-input "l") (setf rot-axis :-x)) ;; -x
       ((equal key-input "f") (setf rot-axis :y)) ;; y
       ((equal key-input "b") (setf rot-axis :-y)) ;; -y
       ((equal key-input "reset")
	(send robot arm :inverse-kinematics
	      (make-coords :pos (send robot arm :end-coords :worldpos))
	      :rotation-axis :z)
	(send *irtviewer* :draw-objects)
	(when real
	  (send *ri* :angle-vector
		(send robot :angle-vector) (* 3 *base-time*)))
q	(return-from rot-hand-vi))
       ((equal key-input "q") (return-from rot-hand-vi)) ;; "q"=quit
       )
      (send robot arm :move-end-rot move-ang rot-axis :world)
      (when move-headp
	(send *hrp2* :head :look-at (send *hrp2* arm :end-coords :worldpos)))
      (send *irtviewer* :draw-objects)
      (when real
	(send *ri* :angle-vector (send robot :angle-vector) move-time))
      )
    ))

(unless (boundp '*hrp2*)
  (print ";; please load setup-hrp2-tomato.l"))
