;;;;hand-camera.l
;;;;JSK Chen 2015...

(load "setup-hrp2-tomato.l")

(defun hand-eye-move()
  
  ;;(loop
   (when
       (send *tfl* :wait-for-transform "BODY" "tomato" (ros::time 0) 10)


      (setq *torso-co* (send *hrp2w* :body_lk :copy-worldcoords))
     ;;first named *torso-co* as the cankao zuobiao....
     ;;wait for transform
     (setq *target-co* (send *tfl* :lookup-transform "BODY" "tomato" (ros::time 0)))
     ;;*target-co* is the coordinate transform
     (setq *ik-target* (send *torso-co* :transform *target-co*))
     ;; *ik-target* is the target center in world woordinate

     (send *hrp2w* :head :look-at (send *ik-target* :worldpos))
     ;; look at the center of branch
     (objects *ik-target*)
     ;; show in IRT viewer....
     (send *ik-target* :replace-rot (send (car (send *hrp2w* :torso)) :copy-rot))
     (setq *hand-eye* (send *ik-target* :copy-coords))

     (send *hand-eye* :locate #f(-450 350 -100))
     ;; almost this distance is ok....
     (setq r (unit-matrix 3))
     (setq r1 (rotate-matrix r -0.59 :z))
     (send *hand-eye* :rotate r1)
     ;;(setq r2 (rotate-matrix r (+ 0 (/ pi 2)) :y))
     (setq r2 (rotate-matrix r (- (/ pi 2) 0.15) :y))
     (setq r3 (rotate-matrix r2 (/ pi 2) :x))
     (send *hand-eye* :rotate r3)

      (setq r1 (rotate-matrix r -0.59 :z))
     (objects *hand-eye*)

     (send *hrp2w* :larm :inverse-kinematics *hand-eye* :rotation-axis t :debug-view t :torso t :link-list (send *hrp2w* :link-list (send *hrp2w* :larm :end-coords :parent) (car (send *hrp2w* :torso :links))))
     ;; Do IK to move the hand....also move the head to look at the branch
     (send *hrp2w* :head :look-at (send *ik-target* :worldpos))
	;;;next we need to setup a command to move the real robot.... reserved...
     (while t
       (setf key-input (read-line))
       (cond
	;;execution
	((equal key-input "x") (send *ri* :angle-vector (send *hrp2w* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)
	 ) ;;

	((equal key-input "r") (send *ri* :angle-vector (send *hrp2w* :reset-manip-pose) 5000)
	 (send *ri* :wait-interpolation)
	 ) ;;

	((equal key-input "g") (move-to-tomato) (return-from hand-eye-move))
	
	((equal key-input "q") (return-from hand-eye-move)) ;; "q"=quit 
	)
       )
     )   ;; do nothing loop.....
  ;; )
  )
