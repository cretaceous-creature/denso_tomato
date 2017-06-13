;; loading 
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
;; robot preparation movement
(defun prep-move
  ()
  (send *hrp2jsk* :reset-manip-pose)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *hrp2jsk* :angle-vector))
  (unix:usleep (* 1000 1000))
  (send *hrp2jsk* :reset-pose)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *hrp2jsk* :angle-vector))
  ;; (unix:usleep (* 1000 1000))
  )

;; robot setup
(defun practice-setup
  ()
  ;; (setup-model-for-hrp2) ; old
  (hrp2jsk-init) ; create instance
  (objects (list *hrp2jsk*)) ; show in IRTViewer
  ;; hrp2 -> hrp2jsk
  (send *hrp2jsk* :fix-leg-to-coords (make-coords))
  (send *hrp2jsk* :reset-pose)
  (send *irtviewer* :draw-objects)
  (unix:usleep (* 200 1000)) ; 0.2 sec
  (prep-move)
  ;; (send *hrp2jsk* :reset-manip-pose)
  ;; (send *irtviewer* :draw-objects)
  ;; (unix:usleep (* 200 1000))
  ;; (send *hrp2jsk* :reset-pose)
  ;; (send *irtviewer* :draw-objects)
  ;; (unix:usleep (* 200 1000))
  ;; (send *hrp2jsk* :init-pose)
  ;; (send *irtviewer* :draw-objects)
  ;; (unix:usleep (* 200 1000))
  ;; environmet setup
  ; (setq *valve* (instance valve :init)) ; make valve
  ; (objects (list *hrp2jsk* *valve*)) ; view models
  ; (send *valve* :locate #f(700 0 1000) :world) ; translate
  ; (send *valve* :orient (deg2rad -90) :y :world) ; rotate
  ;; (send *pickview* :draw-objects) ; old

  ;; new environment setup
  )

;; Error Message
(warn "~%push any key to start~%")
;; wait input key
(read-line)
(warn "Call practice-setup function.
~%practice-setup~%function-name
~%ik-practice~%ik-practice-1~%ik-practice-2~%ik-practice-3~%y-tanaka-ik-practice-1~%")
(warn "~%To send angle vector from this interface to a robot in real or sim,
~%(send *ri* :angle-vector (send *hrp2jsk* :angle-vector))~%")

;; methods of pose
; :reset-pose :reset-manip-pose :init-pose 
; :reset-manip-pose-arm-avoid ;; give :larm or :rarm
; :reset-manip-pose-rarm-avoid ;; not reset :rarm 
; :reset-manip-pose-larm-avoid ;; not reset :larm

;; define function
(defun function-name
  () ; arguments
  (warn "This is a function.~%"))

;; IK practice (Inverse Kinematics)
(defun ik-practice
  ())

;; hpr2-ik-practice-1
(defun ik-practice-1
  ()
  (send *hrp2jsk* ; hrp2 -> hrp2jsk
	:rarm ; right arm
	:inverse-kinematics ; inverse-kinematics
	(make-coords :pos #f(250 -300 1000)) ; make coords
	:rotational-axis nil) ; rotational-axis: yaw-pitch-roll
  (send *irtviewer* :draw-objects)
  ; send angle-vector to a robot in real or sim
  ; (send *ri* :angle-vector (send *hrp2jsk* :angle-vector))
)
;; hrp2jsk-ik-practice-2
; send series of IK and move arm on x-axis
(defun ik-practice-2
  ()
  (dolist ; loop for list ; similar to each-statement of Ruby
      (x (list 250 300 350 400 450 400 350 300 250)) ; list as argument
    ; robot, part, action, pos
    (send *hrp2jsk* :rarm :inverse-kinematics 
	  (make-coords :pos (float-vector x -300 1000)))
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *hrp2jsk* :angle-vector))
    (unix:usleep (* 200 1000))
    ))

;; hrp2jsk-ik-practice-3
; Note: difference between let and let*
(defun ik-practice-3
  ()
  (let* ((start-x 250) ; declaration of local variable
	 (fin-x 600) ; only valid in this body
	 (x-diff 50)
	 (x start-x)) ; refer to start-x ; if let, unable to refer to start-x
    (while t ; while true
      (send *hrp2jsk* :rarm :inverse-kinematics
	    (make-coords :pos (float-vector x -300 1000))
                         :rotational-axis nil)
	    (send *irtviewer* :draw-objects)
	    (unix:usleep (* 200 1000))
            ; (send *ri* :angle-vector (send *hrp2jsk* :angle-vector))
	    (when (= x fin-x) ; evaluate 2nd arg if x=fin-x
	      (return)) ; exit from while loop
	    (setq x (+ x x-diff)) ; assign updated value for x
	    )))

;; unix:command-name ; use unix command from eus
; (unix:usleep (* 200 1000)) ; unix sleep 1000 micro sec * 200 = 0.2sec

(defun practice-reset
  ()
  (send *hrp2jsk* :reset-pose)
  (send *irtviewer* :draw-objects))

;; practice
(defun y-tanaka-ik-practice-1 
  () ; arguments
  (practice-setup)
  ; set up environment

  (let* ((start-x 250) ; declaration of local variable
	 (fin-x 600) ; only valid in this body
	 (x-diff 50)
	 (x start-x)) ; refer to start-x
    (while t ; while true; infinite loop
      (send *hrp2jsk* :rarm :inverse-kinematics
	    (make-coords :pos (float-vector x -300 1000))
	    :rotational-axis nil)
      (send *hrp2jsk* :look-at-hand :rarm)
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *hrp2jsk* :angle-vector))
      (unix:usleep (* 200 1000))
      (when (= x fin-x) ; evaluate 2nd arg if x=fin-x
	(return)) ; exit from while loop
      (setq x (+ x x-diff))
      )))

;; memo
; (function)
; (quote ) -> '
; :keyword キーワード引数,関数の動作を変更できる．
; send関数にキーワードを付加している．
; (send *object-name* :keyword (k e y w o r d a r g s))
; シンボル：値比較で等しくなる
; つまり同じメモリ領域の値にセットされている
; let, let*の違い
; *instance*b
; :method
; defvar, setqの違い
; :copy-worldcoordsの機能
; make-coordsの使い方
; :rotation-axisの使い方
; xtionで点群表示
; 点群の座標に対してIKを解いて手を伸ばす
; 1. バルブ単体で回してみる
; 2. 手先でバルブを回してみる
; 3. 腕の関節を任意で固定してバルブを回してみる
; 4. 認識を入れる?