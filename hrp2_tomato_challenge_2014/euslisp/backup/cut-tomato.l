;;;;cut-tomato.l
;;;;JSK Chen 2015...

;(load "setup-hrp2-tomato.l")
(load "hand-camera.l")


(defun pr()
  (send *ri* :angle-vector (send *hrp2w* :reset-manip-pose) 5000)
  )


(defun hasami()
   (while t
	(setf key-input (read-line))
	(cond
	 ;; move-hand-vi   adjusting, the maybe error
	 ((equal key-input "1") (start-cut)
	  ) ;;
	 ((equal key-input "2") (stop-cut)
	  ) ;;
	 ((equal key-input "3") (start-left-cut)
	  ) ;;
	 ((equal key-input "4") (stop-left-cut)
	  ) ;;
	 ((equal key-input "q") (return-from hasami)) ;; "q"=quit
	 )
	)
 )
(defun move-to-tomato()
  
  (when
      (send *tfl* :wait-for-transform "BODY" "pcenter" (ros::time 0) 10)
    (setq *tomato-up-t* (send *tfl* :lookup-transform "BODY" "pcenter" (ros::time 0)))
    (when
	(send *tfl* :wait-for-transform "BODY" "ppedicel" (ros::time 0) 10)

      ;; (setq *torso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))
      (setq *torso-co* (send *hrp2w* :body_lk :copy-worldcoords))
      ;;first named *torso-co* as the cankao zuobiao....
      ;;wait for transform
      
      (setq *tomato-pedicel-t* (send *tfl* :lookup-transform "BODY" "ppedicel" (ros::time 0)))
      ;;*tomato-up tomato-pedicel* is the coordinate transform
      (setq *tomato-up* (send *torso-co* :transform *tomato-up-t*))
      (setq *torso-co* (send *hrp2w* :body_lk :copy-worldcoords))
      (setq *tomato-pedicel* (send *torso-co* :transform *tomato-pedicel-t*))
      ;; *ik-target* is the target center in world woordinate

      (send *hrp2w* :head :look-at (send *tomato-up* :worldpos)) ;;no need to look
      
      (objects *tomato-up*)
      (objects *tomato-pedicel*)

      (send *ri* :angle-vector (send *hrp2w* :reset-manip-pose) 3000)
      (send *ri* :wait-interpolation)
      
      ;; show in IRT viewer.... copy rot
      (send *tomato-pedicel* :replace-rot (send  *hrp2w* :body_lk :copy-rot))
      (send *tomato-up* :replace-rot (send  *hrp2w* :body_lk :copy-rot))

      ;;(send *tomato-pedicel* :locate #f(-100 20 0))
      
      ;; now yaw to let the x axis aim at pedicel in xy plane
      (setq r (unit-matrix 3))
      (setq x-vec (- (elt (send *tomato-pedicel* :worldpos) 0) (elt (send *tomato-up* :worldpos) 0)))
      (setq y-vec (- (elt (send *tomato-pedicel* :worldpos) 1) (elt (send *tomato-up* :worldpos) 1)))
      (setq z-vec (- (elt (send *tomato-pedicel* :worldpos) 2) (elt (send *tomato-up* :worldpos) 2)))
      (setq theta-yaw (atan2 y-vec x-vec))
      (setq theta-roll (atan2 z-vec y-vec))
      ;; rotate theta ....it should be x axis aim at pedicel   
      (setq r1 (rotate-matrix r theta-yaw :z))
      (send *tomato-up* :rotate r1)
      (setq theta-pitch (atan2 (- 0 z-vec) (abs x-vec)))
      ;;now pitch , to let the x axis aim at pedicel in xz plane
      (setq r2 (rotate-matrix r  theta-pitch :y))

      (setq theta-roll (atan2 z-vec y-vec))
      (setq r3 (rotate-matrix r theta-roll :x))

      ;;pitch to the final pose
      (send *tomato-up* :rotate r2)
      
      ;;then we need to move to pedicel location....thats all
      ;; we need to use theta_yaw to decide which hand to use
      ;; and different hand has different locate traslate
      ;;  (send *tomato-up* :locate #f(-120 20 0)) for right
      ;; (send *tomato-up* :locate #f(-120 20 20)) for left      
      
      (setq *ik-target* (send *tomato-up* :copy-coords))
      (if (>= theta-yaw 0)
	  (progn
	    (setq arm :rarm)
	    ;; because of robot   we cant handle all the situation
	    (if (> (abs theta-yaw) (/ pi 2.2))
		(progn
		   (if (> (abs theta-yaw) (/ pi 1.3))
		       (setq r-tmp (rotate-matrix r (/ (- 0 pi) 1.4) :z))
		     (setq r-tmp (rotate-matrix r (/ (- 0 pi) 2.6) :z))
		     )
		  (send *tomato-up* :rotate r-tmp)
		  (setq *ik-target* (send *tomato-up* :copy-coords))
		  ;;then transform to the center...
		  (send *tomato-pedicel* :transform (make-coords :pos (float-vector (- 0 (/ x-vec 2)) (- 0 (/ y-vec 2)) (- 0 (/ z-vec 2)))))
		  (send *ik-target* :replace-pos (send *tomato-pedicel* :copy-pos))
		   (send *tomato-pedicel* :locate #f(20 5 0))
		  )
	      (progn
		(if(eq theta-yaw 0.0)   ;;only one tomato use right hand
		    (progn
		      (setq r-tmp (rotate-matrix r (/ pi 4) :z))
		      (send *tomato-up* :rotate r-tmp)
		      (setq r-tmp (rotate-matrix r (/ pi 8) :y))
		      (send *tomato-up* :rotate r-tmp)
		      (send *tomato-up* :locate #f(-25 0 10))
		      (setq *ik-target* (send *tomato-up* :copy-coords))
		      )
		  )
		)
	      )
	    (send *ik-target* :locate #f(-140 -15 36))
	    )
	(progn
	  (setq arm :larm)
	  (if (> (abs theta-yaw) (/ pi 2.2))
	      (progn
		(if (> (abs theta-yaw) (/ pi 1.3))
		    (setq r-tmp (rotate-matrix r (/ pi 1.4) :z))
		  (setq r-tmp (rotate-matrix r (/ pi 2.6) :z))
		  )
		(send *tomato-up* :rotate r-tmp)
		(setq *ik-target* (send *tomato-up* :copy-coords))
		(send *tomato-pedicel* :transform (make-coords :pos (float-vector (- 0 (/ x-vec 2)) (- 0 (/ y-vec 2)) (- 0 (/ z-vec 2)))))
		(send *ik-target* :replace-pos (send *tomato-pedicel* :copy-pos))
		(send *tomato-pedicel* :locate #f(20 -5 0))
		)
	    )
	  (send *ik-target* :locate #f(-140 35 45))
	  )
	)
      ;;first ik-target
      (send *hrp2w* arm :inverse-kinematics *ik-target* :rotation-axis t :debug-view t :torso t :link-list (send *hrp2w* :link-list (send *hrp2w* arm :end-coords :parent) (car (send *hrp2w* :torso :links))))
      ;; Do IK to move the hand....also move the head to look at the branch
      (send *hrp2w* :head :look-at (send *tomato-up* :worldpos))
	;;;next we need to setup a command to move the real robot.... reserved...
      (while t
	(setf key-input (read-line))
	(cond
	 ;;execution
	 ((equal key-input "x")
	  (send *hrp2w* arm :inverse-kinematics *ik-target* :rotation-axis t :debug-view t :torso t :link-list (send *hrp2w* :link-list (send *hrp2w* arm :end-coords :parent) (car (send *hrp2w* :torso :links))))
	  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
	  (send *ri* :wait-interpolation)
	  ) 

	 ;; move-hand-vi   adjusting, the maybe error
	 ((equal key-input "m") (move-hand-vi)
	  ) ;;

	 ;;reset-pose   very important
	 ((equal key-input "r") (send *ri* :angle-vector (send *hrp2w* :reset-manip-pose) 5000)
	  (send *ri* :wait-interpolation)
	  ) ;;

	 ;; cutting tomatoes, if success, should reset-pose
	 ((equal key-input "c")
	  	  (if (>= theta-yaw 0)
		      (start-cut)
		    (start-left-cut)
		    )
		  )

	 ((equal key-input "s")
	  (if (>= theta-yaw 0)
	      (stop-cut)
	    (stop-left-cut)
	    )
	  )

	 ;;placing tomatoes, should follow reset-pose
	 ((equal key-input "p")
	  (reset-place-pose arm)
	  )

	 ;;quit
	 ((equal key-input "q") (return-from move-to-tomato)) ;; "q"=quit

	 ;;next we move to pedicel pos  only pos, rot do no change
	 ((equal key-input "g")
	  (send *ik-target* :replace-pos (send *tomato-pedicel* :copy-pos))
	  (if (>= theta-yaw 0)
	      (progn
		(setq arm :rarm)
		(if (eq theta-yaw 0)
		    (send *ik-target* :locate #f(0 10 0))
		    )
		(send *ik-target* :locate #f(-115 -15 35))
		)
	    (progn
	      (setq arm :larm)
	      (send *ik-target* :locate #f(-118 40 45))
	      )
	    )
	  (send *hrp2w* arm :inverse-kinematics *ik-target* :rotation-axis t :debug-view t :torso t :link-list (send *hrp2w* :link-list (send *hrp2w* arm :end-coords :parent) (car (send *hrp2w* :torso :links))))
	  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 2000)
	  (send *ri* :wait-interpolation)
	  )
	 )
	)
      )  
    )
