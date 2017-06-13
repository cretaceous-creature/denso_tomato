#!/usr/bin/env roseus
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
; eusLisp robot interface

;; methods of pose
; :reset-pose :reset-manip-pose :init-pose
; :reset-manip-pose-arm-avoid ;; give :larm or :rarm
; :reset-manip-pose-rarm-avoid ;; not reset :rarm
; :reset-manip-pose-larm-avoid ;; not reset :larm

;; define function
(defun function-name
  () ; arguments
  (warn "This is a function.~%")
)

;; robot setup
(defun practice-setup
  ()
  ; (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
  (hrp2w-init)
  (objects *hrp2w*)
  ;(send *hrp2w* :init-pose)
  ;(send *irtviewer* :draw-objects)
  (send *hrp2w* :reset-pose)
  ;(send *irtviewer* :draw-objects)
  (send *hrp2w* :reset-manip-pose)
  (send *hrp2w* :fix-leg-to-coords (make-coords))
  ; (valve-setup)
  (send *irtviewer* :draw-objects) ; update view instead of click
)

(defun test-joint-command
  ()
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 3000)
  (send *ri* :wait-interpolation))

(defun valve-setup
  () ;; old environmet setup for valve
  (setq *valve* (instance valve :init)) ; make valve
  (objects (list *hrp2w* *valve*)) ; view models
  (send *valve* :locate #f(700 0 1000) :world) ; translate
  (send *valve* :orient (deg2rad -90) :y :world) ; rotate
)

(defun get-co
  ()
  (setq *co* (send (send *hrp2w* :rarm :end-coords) :copy-worldcoords)))

;; IK practice (Inverse Kinematics)
(defun ik-practice
  (pos)
  (setq *co* (send (send *hrp2w* :rarm :end-coords) :copy-worldcoords))
  (send *co* :translate #f(250 0 0)) ; needs constrants for y and z
  (send *hrp2w* ; hrp2w
	:rarm ; right arm
	:inverse-kinematics ; inverse-kinematics
	(make-coords :pos pos) ; make coords
	:rotational-axis nil)) ; rotational-axis: yaw-pitch-roll
; difference between float-vector and #f

(defun neck-move
  ()
  (send *hrp2w* :head :look-at (send *hrp2w* :rarm :end-coords :pos))
  (send *hrp2w* :head :neck-p :joint-angle 0.0)
  )

;; hpr2-ik-practice-1
;; send series of IK and move arm on x-axis
(defun ik-practice-1
  ()
)

;; hrp2w-ik-practice-2
(defun ik-practice-2
  ()
  (dolist ; loop for list ; similar to each-statement of Ruby
      (x (list 250 300 350 400 350 300 250)) ; list as argument
    (send *hrp2w* :rarm :inverse-kinematics
	  (make-coords :pos (float-vector x -300 1000)) :debug-view t)
    (send *irtviewer* :draw-objects)
    (unix:usleep (* 200 1000))
    ))

;; hrp2w-ik-practice-3
; Note: difference between let and let*
(defun ik-practice-3
  ()
  (let* ((start-x 250) ; declaration of local variable
	 (fin-x 600) ; only valid in this body
	 (x-diff 50)
	 (x start-x)) ; refer to start-x ; if let, unable to refer to start-x
    (while t ; while true
      (send *hrp2w* :rarm :inverse-kinematics
	    (make-coords :pos (float-vector x -300 1000)))
                         :rotational-axis nil)
	    (send *irtviewer* :draw-objects)
	    (unix:usleep (* 200 1000))
	    (when (= x fin-x) ; evaluate 2nd arg if x=fin-x
	      (return)) ; exit from while loop
	    (setq x (+ x x-diff))
	    ))))

(defun tf-setup ()
  (setq *tfl* (instance ros::transform-listener :init))
  (send *tfl* :wait-for-transform "/CHEST_LINK0"
        "/camera_remote/depth_registered/cluster_decomposeroutput01" (ros::time 0) 1)
  ;; (send *tfl* :lookup-transform "/CHEST_LINK0"
  ;;       "/camera_remote/depth_registered/cluster_decomposeroutput01"
  ;;       (ros::time 0))
  )

(defun tf-ik ()
  (setq *torso-co* (send (car send *hrp2w* :torso)) :copy-worldcoords)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *target-co*
        (send *tfl* :lookup-transform
              "/CHEST_LINK0"
              "/camera_remote/depth_registered/cluster_decomposeroutput01"
              (ros::time 0)))
  (setq *ik-target* (send *torso-co* :transform *target-co*))
  )

;; show instruction
(warn "~%push any key to start~%") ; Error Message
(read-line) ; wait input key
(warn "Call practice-setup function.
~%practice-setup~%function-name~%ik-practice
~%ik-practice-1~%ik-practice-2~%ik-practice-3
~%y-tanaka-ik-practice-1~%")

;; unix:command-name ; use unix command from eus
(unix:usleep (* 200 1000)) ; unix sleep 1000 micro sec * 200 = 0.2sec

;; practice
(defun y-tanaka-ik-practice-1 
  () ; arguments
  (setup-model-for-hrp2w) ; make hrp2w
  (send *hrp2w* :reset-pose)
  (send *hrp2w* :fix-leg-to-coords (make-coords))

  ; set up environment
  (let* ((start-x 250) ; declaration of local variable
	 (fin-x 600) ; only valid in this body
	 (x-diff 50)
	 (x start-x)) ; refer to start-x
    (while t ; while true; infinite loop
      (send *hrp2w* :rarm :inverse-kinematics
	    (make-coords :pos (float-vector x -300 1000))
	    :rotational-axis nil)
      (send *irtviewer* :draw-objects)
      (unix:usleep (* 200 1000))
      (when (= x fin-x) ; evaluate 2nd arg if x=fin-x
	(return)) ; exit from while loop
      (setq x (+ x x-diff))
      ))
  )
)

;; memo
; (function)
; let, let*の違い
; *instance*
; :method
; defvar, setqの違い
; :copy-worldcoordsの機能
; ikを解く時,world-coords
; make-coordsの使い方
; :rotation-axisの使い方
; xtionで点群表示
; 点群の座標に対してIKを解いて手を伸ばす
; 1. バルブ単体で回してみる
; 2. 手先でバルブを回してみる
; 3. 腕の関節を任意で固定してバルブを回してみる
; 4. 認識を入れる?

; make-cube
; (setq *cube* (make-cube 100 100 100))