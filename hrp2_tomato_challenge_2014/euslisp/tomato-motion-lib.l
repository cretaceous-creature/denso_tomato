;;
;; tomato-motion-lib.l
;;

(defun start-cut ()
  (cond
   ((boundp '*hrp2w*)
    ;; hrp2008
    (hrp2w-start-grasp :rarm :angle -8))
   ((boundp '*aria*)
    ;; c2 (bounding *aria*)
    (c2-start-cut))
   ))

(defun stop-cut ()
  (cond
   ((boundp '*hrp2w*)
    ;; hrp2008
    (hrp2w-stop-grasp :rarm))
   ((boundp '*aria*)
    ;; c2 (bounding *aria*)
    (c2-stop-cut))
   ))

(defun move-hand-vi
  (&key (real *real*)
	(robot *hrp2*)
	(arm :rarm)
	(move-time (* 0.5 *base-time*))
	(move-dist 20)
	(move-headp t))
  (let (key-input
	velocity (short-dist (/ move-dist 3.0))
	(world-co (make-coords))
	(forward-dir (float-vector 1 0 0))
	(left-dir (float-vector 0 1 0))
	(up-dir (float-vector 0 0 1))
	)
    (when move-headp
      (send *hrp2* :head :look-at (send *hrp2* arm :end-coords :worldpos)))
    (when real
      (cond
       ((boundp '*hrp2*)
	(send *ri* :angle-vector (send robot :angle-vector)
	      (* 3 *base-time*)))
       ((boundp '*aria*)
	(model2real :sleep-time (* 3 *base-time*)))
       ))
    (when (boundp '*aria*)
      (setq world-co (send (caddr (send *robot* :torso)) :copy-worldcoords))
      (setq forward-dir (send world-co :rotate-vector forward-dir)
	    left-dir (send world-co :rotate-vector left-dir)
	    up-dir (send world-co :rotate-vector up-dir))
    (while t
      (setf key-input (read-line))
      (cond
<<<<<<< .mine
       ((equal key-input "s") (setf velocity (float-vector 0 0 (- move-dist)))) ;; "j"=down
       ((equal key-input "w") (setf velocity (float-vector 0 0 move-dist))) ;; "k"=up
       ((equal key-input "a") (setf velocity (float-vector 0 move-dist 0))) ;; "h"=left
       ((equal key-input "d") (setf velocity (float-vector 0 (- move-dist) 0))) ;; "l"=right
       ((equal key-input "c") (setf velocity (float-vector move-dist 0 0))) ;; "f"=forward
       ((equal key-input "x") (setf velocity (float-vector (- move-dist) 0 0))) ;; "b"=backward
=======
       ((equal key-input "j")
	(setf velocity (scale (- move-dist) up-dir))) ;; "j"=down
       ((equal key-input "k")
	(setf velocity (scale move-dist up-dir))) ;; "k"=up
       ((equal key-input "h")
	(setf velocity (scale move-dist left-dir))) ;; "h"=left
       ((equal key-input "l")
	(setf velocity (scale (- move-dist) left-dir))) ;; "l"=right
       ((equal key-input "f")
	(setf velocity (scale move-dist forward-dir))) ;; "f"=forward
       ((equal key-input "b")
	(setf velocity (scale (- move-dist) forward-dir))) ;; "b"=backward
>>>>>>> .r65681

       ((equal key-input "jj")
	(setf velocity (scale (- short-dist) up-dir))) ;; "j"=down
       ((equal key-input "kk")
	(setf velocity (scale short-dist up-dir))) ;; "k"=up
       ((equal key-input "hh")
	(setf velocity (scale short-dist left-dir))) ;; "h"=left
       ((equal key-input "ll")
	(setf velocity (scale (- short-dist) left-dir))) ;; "h"=left
       ((equal key-input "ff")
	(setf velocity (scale short-dist forward-dir))) ;; "f"=forward
       ((equal key-input "bb")
	(setf velocity (scale (- short-dist) forward-dir))) ;; "b"=backward

       ((equal key-input "q") (return-from move-hand-vi)) ;; "q"=quit
       )
      (send robot arm :move-end-pos velocity :world :rotation-axis :z)
      (when move-headp
	(send *hrp2* :head :look-at (send *hrp2* arm :end-coords :worldpos)))
      (send *irtviewer* :draw-objects)
      (when real
	(cond
	 ((boundp '*hrp2*)
	  (send *ri* :angle-vector (send robot :angle-vector) move-time))
	 ((boundp '*aria*)
	  (model2real :sleep-time move-time))
	 ))
      )
    ))

(defun rot-hand-vi
  (&key (real *real*)
	(robot *hrp2*)
	(arm :rarm)
	(move-time (* 0.5 *base-time*))
	(move-ang 3)
	(move-headp t))
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
       ((equal key-input "h") (setf rot-axis :-x)) ;; x
       ((equal key-input "l") (setf rot-axis :x)) ;; -x
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
	(return-from rot-hand-vi))
       ((equal key-input "q") (return-from rot-hand-vi)) ;; "q"=quit
       )
      ;;(send robot arm :move-end-rot move-ang rot-axis :world)
      (send robot arm :move-end-rot move-ang rot-axis :local)
      (when move-headp
	(send *hrp2* :head :look-at (send *hrp2* arm :end-coords :worldpos)))
      (send *irtviewer* :draw-objects)
      (when real
	(send *ri* :angle-vector (send robot :angle-vector) move-time))
      )
    ))

(unless (boundp '*hrp2*)
  (print ";; to use hrp2, please load setup-hrp2-tomato.l"))
(unless (boundp '*aria*)
  (print ";; to use c2, please load setup-c2-tomato.l"))

