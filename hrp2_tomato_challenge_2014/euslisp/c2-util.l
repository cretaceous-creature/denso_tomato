(defun c2-util-init ()
  (ros::advertise "/scissors_controller/set_angle" std_msgs::Float64 1)
  (ros::advertise "/c2/cmd_vel" geometry_msgs::Twist 1)
  )

;; scissors is fixed with 45 degree slant
(defun c2-reset-manip-pose ()
  ;; (send *aria* :reset-manip-pose)
  ;; (send *aria* :arm_l_joint6 :joint-angle 45)
  ;; (send *aria* :neck3 :joint-angle -15)
  ;; (send *irtviewer* :draw-objects)
  ;; (model2real :sleep-time 3000))
  (c2-reset-tomato-pose))

(defun c2-swing-waist (angle)
  (send *aria* :hip2 :joint-angle angle)
  (send *irtviewer* :draw-objects)
  (model2real :sleep-time 3000))

(defun c2-set-finger-angle (angle)
  (let (fing-msg)
    (setq fing-msg (instance std_msgs::Float64 :init))
    (send fing-msg :data angle)
    (ros::publish "/scissors_controller/set_angle" fing-msg)
    )
  )

(defun c2-start-cut ()
  (c2-set-finger-angle 0.2))

(defun c2-stop-cut ()
  (c2-set-finger-angle 0.74))

(defun c2-cmd-vel (x y theta)
  (let (vel-msg)
    (setq vel-msg (instance geometry_msgs::Twist :init))
    (send vel-msg :linear :x x)
    (send vel-msg :linear :y y)
    (send vel-msg :angular :z theta)
    (ros::publish "/c2/cmd_vel" vel-msg)
    )
  )

(defun c2-move-stop ()
  (c2-cmd-vel 0 0 0))
;; move forawrd and back: move about 0.08[m] with defualt sleep
(defun c2-move-forward (&optional (msleep 1000))
  (c2-cmd-vel 0.1 0 0)
  (unix:usleep (* 1000 msleep))
  (c2-move-stop))
(defun c2-move-back (&optional (msleep 1000))
  (c2-cmd-vel -0.1 0 0)
  (unix:usleep (* 1000 msleep))
  (c2-move-stop))

;; move right and left: DANGER!
(defun c2-move-right (&optional (msleep 1000))
  (c2-cmd-vel 0 -0.1 0)
  (unix:usleep (* 1000 msleep))
  (c2-move-stop))
(defun c2-move-left (&optional (msleep 1000))
  (c2-cmd-vel 0 0.1 0)
  (unix:usleep (* 1000 msleep))
  (c2-move-stop))

;; turn right and left: move about 15[deg] with default sleep
(defun c2-turn-right (&optional (msleep 1000))
  (c2-cmd-vel 0 0 -0.2)
  (unix:usleep (* 1000 msleep))
  (c2-move-stop))
(defun c2-turn-left (&optional (msleep 1000))
  (c2-cmd-vel 0 0 0.2)
  (unix:usleep (* 1000 msleep))
  (c2-move-stop))

(defun c2-switch-back ()
  (c2-turn-right 1200)
  (unix:usleep (* 500 1000))
  (c2-move-back)
  (unix:usleep (* 500 1000))
  (c2-turn-left 1200)
  (unix:usleep (* 500 1000))
  (c2-move-forward)
  )


;; move hand
(defun c2-move-rhand-vi ()
  (move-hand-vi :robot *aria* :arm :rarm :move-headp nil))
(defun c2-move-lhand-vi ()
  (move-hand-vi :robot *aria* :arm :larm :move-headp nil))


;; motion
(defun c2-put-tomato (&optional (sleep-time 8000))
  (let ((larm-av
	 (float-vector -9.40203 66.5168 -0.99464 -99.9737 -15.9299 7.71617 16.8708 21.4408)))
    (send *aria* :larm :angle-vector larm-av)
    ;;(send *aria* :hip2 :joint-angle 0.0)
    (send *aria* :hip2 :joint-angle -90.0)
    (send *aria* :neck3 :joint-angle -60.0)
    (model2real :sleep-time sleep-time)
    )
  )
(defun c2-reset-tomato-pose (&optional (sleep-time 8000))
  (send *aria* :reset-manip-pose)
  (send *aria* :neck3 :joint-angle -15.0)
  (send *aria* :arm_l_joint6 :joint-angle 45)
  (send *aria* :hip2 :joint-angle 90.0)
  (model2real :sleep-time sleep-time)
  )

(defun c2-reset-larm (&optional (sleep-time 3000))
  (let ((larm-av
	 (float-vector 0.0 -2.21381 45.0 -120.0 -0.071416 4.33423 30.0 -0.598775)))
    (send *aria* :larm :angle-vector larm-av)
    (model2real :sleep-time sleep-time)
    )
  )

(defun c2-reset-rarm (&optional (sleep-time 3000))
  (let ((rarm-av
	 (float-vector 0.0 0.89541 0.499902 -120.0 0.049437 2.87301 -30.0 2.09296)))
    (send *aria* :rarm :angle-vector rarm-av)
    (model2real :sleep-time sleep-time)
    )
  )


;; manip with (send *aria* :arm_l_joint6 :joint-angle 45)

(defun c2-reset-manip-larm (&optional (sleep-time 3000))
  (let ((larm-av
	 ;;(float-vector 1.061521e-14 -5.90661 76.6804 -126.404 13.5179 42.3369 -0.20602 -36.3012)))
	 (float-vector 1.061521e-14 -30 76.6804 -126.404 13.5179 42.3369 -0.20602 -36.3012)))
    (send *aria* :larm :angle-vector larm-av)
    (model2real :sleep-time sleep-time)
    )
  )

(defun c2-reset-manip-rarm (&optional (sleep-time 3000))
  (let ((rarm-av
	 (float-vector 3.900901e-15 -40 -5.22128 -98.8 -18.1421 -56.2072 10.2492 35.8038)))
    (send *aria* :rarm :angle-vector rarm-av)
    (model2real :sleep-time sleep-time)
    )
  )

(defun c2-neck-p (angle &optional (sleep-time 2000))
  (send *aria* :neck3 :joint-angle angle)
  (model2real :sleep-time sleep-time))

(defun c2-neck-r (angle &optional (sleep-time 2000))
  (send *aria* :neck1 :joint-angle angle)
  (model2real :sleep-time sleep-time))

(defun c2-neck-r-p (roll pitch &optional (sleep-time 2000))
  (send *aria* :neck1 :joint-angle roll)
  (send *aria* :neck3 :joint-angle pitch)
  (model2real :sleep-time sleep-time))
