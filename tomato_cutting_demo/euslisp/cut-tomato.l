;;;;cut-tomato.l
;;;;JSK Chen 2015...

;(load "setup-hrp2-tomato.l")
(load "hand-camera.l")


(defun send-reset-manip-pose (&key (time 5000))
  (send *hrp2w* :reset-manip-pose)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) time)
  (send *ri* :wait-interpolation))
(defun pr () (send-reset-manip-pose))

(defun send-angle-vector (&key (time 5000))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) time)
  (send *ri* :wait-interpolation))

(defun move-ik (co &key (arm :rarm) (time 5000))
  (when
      (send *hrp2w* arm :inverse-kinematics co
	    :rotation-axis t :debug-view t :torso t
	    :link-list (send *hrp2w* :link-list
			     (send *hrp2w* arm :end-coords :parent)
			     (car (send *hrp2w* :torso :links))))
    (send-angle-vector :time time)
    )
  (send *irtviewer* :draw-objects)
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
	 ((equal key-input "3") (start-cut :arm :larm)
	  ) ;;
	 ((equal key-input "4") (stop-cut :arm :larm)
	  ) ;;
	 ((equal key-input "q") (return-from hasami)) ;; "q"=quit
	 )
	)
 )

(defun lookup-tomato-tf ()
  (setq *tomato-up-t* nil
	*tomato-pedicel-t* nil)
  (when
      (send *tfl* :wait-for-transform "BODY" "pcenter" (ros::time 0) 10)
    (setq *tomato-up-t*
	  (send *tfl* :lookup-transform "BODY" "pcenter" (ros::time 0))))
  (when
      (send *tfl* :wait-for-transform "BODY" "ppedicel" (ros::time 0) 10)
    (setq *tomato-pedicel-t*
	  (send *tfl* :lookup-transform "BODY" "ppedicel" (ros::time 0))))
  (cons *tomato-up-t* *tomato-pedicel-t*)
  )


(defun move-to-tomato()  
  (lookup-tomato-tf)
  (when (and *tomato-up-t* *tomato-pedicel-t*)
    (send *hrp2w* :reset-manip-pose)
    ;; (setq *torso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))
    (setq *torso-co* (send *hrp2w* :body_lk :copy-worldcoords))
    ;;first named *torso-co* as the reference coordinate....
    
    ;;*tomato-up tomato-pedicel* is the coordinate transform
    (setq
     *tomato-up*
     (send (send *torso-co* :copy-worldcoords) :transform *tomato-up-t*)
     *tomato-pedicel*
     (send (send *torso-co* :copy-worldcoords) :transform *tomato-pedicel-t*))
    ;; *ik-target* is the target center in world coordinate
    
    ;; show in IRT viewer.... copy rot
    (send *tomato-pedicel* :replace-rot (send *torso-co* :copy-rot))
    (send *tomato-up* :replace-rot (send  *torso-co* :copy-rot))
    ;;(send *tomato-pedicel* :locate #f(-100 20 0))

    ;; now yaw to let the x axis aim at pedicel in xy plane
    (setq vec (v- (send *tomato-pedicel* :worldpos)
		  (send *tomato-up* :worldpos)))
    (setq x-vec (elt vec 0)
	  y-vec (elt vec 1)
	  z-vec (elt vec 2))
    (setq theta-roll (atan2 z-vec y-vec)
	  theta-pitch (atan2 (- z-vec) (abs x-vec))
	  theta-yaw (atan2 y-vec x-vec))

    ;; rotate theta ....it should be x axis aim at pedicel   
    (send *tomato-up* :rotate theta-yaw :z)
    ;;now pitch , to let the x axis aim at pedicel in xz plane
    ;;pitch to the final pose
    (send *tomato-up* :rotate theta-pitch :y)
    
    ;;no need to look
    (send *hrp2w* :head :look-at (send *tomato-up* :worldpos))
    (objects (list *hrp2w* *tomato-up* *tomato-pedicel*))
    ;; (send-reset-manip-pose :time 3000)
    (send-angle-vector :time 3000)


    ;;then we need to move to pedicel location....thats all
    ;; we need to use theta_yaw to decide which hand to use
    ;; and different hand has different locate traslate
    ;;  (send *tomato-up* :locate #f(-120 20 0)) for right
    ;; (send *tomato-up* :locate #f(-120 20 20)) for left      

    (setq singletomatop (eps= theta-yaw 0.0))
    ;;only one tomato use right hand
    (if singletomatop
	(progn
	  (setq arm :rarm
		harvestablep t))
      (progn
	(setq arm (if (>= theta-yaw 0.0) :rarm :larm)
	      harvestablep (> (abs theta-yaw) (/ pi 2.2)))
	;; because of robot   we cant handle all the situation
	))

    (unless harvestablep
      (format t ";; cannot harvest: pedicel angle too deep.~%")
      (return-from move-to-tomato nil))

    (setq *approach-target* (send *tomato-up* :copy-coords)
	  *pedicel-target* (send *tomato-pedicel* :copy-coords)
	  *cut-target* (make-coords))

    (cond
     (singletomatop
      (send *approach-target* :rotate (/ pi 4) :z)
      (send *approach-target* :rotate (/ pi 8) :y)
      ;; considering pedicel
      (send *approach-target* :locate (float-vector -25 0 10))
      (send *approach-target* :locate (float-vector -140 -15 36))
      )
     ((eq arm :rarm)
      ;; because of robot   we cant handle all the situation
      (if (> (abs theta-yaw) (/ pi 1.3))
	  (send *approach-target* :rotate (/ -pi 1.4) :z)
	(send *approach-target* :rotate (/ -pi 2.6) :z))
      ;;then transform to the center...
      (send *pedicel-target* :translate (scale -0.5 vec))
      (send *approach-target* :replace-pos
	    (send *pedicel-target* :copy-pos))
      (send *pedicel-target* :locate (float-vector 20 5 0))
      (send *approach-target* :locate (float-vector -140 -15 36))
      )
     ((eq arm :larm)
      (if (> (abs theta-yaw) (/ pi 1.3))
	  (send *approach-target* :rotate (/ pi 1.4) :z)
	(send *approach-target* :rotate (/ pi 2.6) :z))
      (send *pedicel-target* :translate (scale -0.5 vec))
      (send *approach-target* :replace-pos
	    (send *pedicel-target* :copy-pos))
      (send *pedicel-target* :locate (float-vector 20 -5 0))
      (send *approach-target* :locate (float-vector -140 35 45))
      )
     )

    (setq *cut-target* (send *approach-target* :copy-worldcoords))
    (send *cut-target* :replace-pos (send *pedicel-target* :copy-pos))
    (cond
     (singletomatop
      (send *cut-target* :locate (float-vector -115 -5 35))
      )
     ((eq arm :rarm)
      (send *cut-target* :locate (float-vector -115 -15 35))
      )
     ((eq arm :larm)
      (send *cut-target* :locate (float-vector -118 40 45))
      )
     )

    ;;first approach-target
    (send *hrp2w* arm :inverse-kinematics *approach-target*
	  :rotation-axis t :debug-view t :torso t
	  :link-list (send *hrp2w* :link-list
			   (send *hrp2w* arm :end-coords :parent)
			   (car (send *hrp2w* :torso :links))))
    ;; Do IK to move the hand....also move the head to look at the branch
    (send *hrp2w* :head :look-at (send *tomato-up* :worldpos))
    ;; next we need to setup a command to move the real robot.... reserved...

    (while t
      (setf key-input (read-line))
      (cond
       ;;execution
       ((equal key-input "x")
	(move-ik *approach-target* :arm arm)
	) 

       ;; move-hand-vi   adjusting, the maybe error
       ((equal key-input "m") (move-hand-vi)
	) ;;

       ;;reset-pose   very important
       ((equal key-input "r")
	(send-reset-manip-pose)
	) ;;

       ;; cutting tomatoes, if success, should reset-pose
       ((equal key-input "c")
	(start-cut :arm arm)
	)

       ((equal key-input "s")
	(stop-cut :arm arm)
	)

       ;;placing tomatoes, should follow reset-pose
       ((equal key-input "p")
	(reset-place-pose arm)
	)

       ;;quit
       ((equal key-input "q") (return-from move-to-tomato)) ;; "q"=quit

       ;;next we move to pedicel pos  only pos, rot do no change
       ((equal key-input "g")
	(move-ik *cut-target* :arm arm :time 2000)
	)
       )
      )  ;; while loop
    ) ;; when
  )  ;; defun
