(load "setup-hrp2-tomato.l")

(setf *picking-counter* 1)

;; global variable set default to 30
(defvar default-start 30)
(defvar *padding* 10)
(defvar *hasami-lenght* 70)

(setf y-lenght default-start)
(setf x-lenght default-start)


;; hako center position setup from chen's code
;;<<<<<<< .mine
;; (setq *hako-center-position* (send *hako* :copy-worldcoords))
;;=======
;; (setq *hako-center-position* (send hako :copy-worldcoords))
;;>>>>>>> .r65311

(setf *hako-center-position* (float-vector 240 -20 840))
;; size of hako associated with hrp2
(setf *hako-measurements* (float-vector 220 340 240))


;; set up fixed size variables
(defvar *tomato-size* 50)
(defvar *hako-y-lenght* (elt *hako-measurements* 1))
(defvar *hako-x-width* (elt *hako-measurements* 0))



;;---------------------------------------------------
;; function to increment placement to next position
;;--------------------------------------------------
(defun tomato-arrangement-position ()
  (when (< y-lenght (- *hako-y-lenght* (/ *tomato-size* 2)))
    (setf y-lenght (+ (+ *tomato-size* *padding*) y-lenght)))

  (when (>= y-lenght (- *hako-y-lenght* (/ *tomato-size* 2)))
    (if(>= (+ x-lenght *padding* ) *hako-x-width*)
        (setf x-lenght default-start))
    (setf y-lenght default-start)
    (setf x-lenght (+ (+ *tomato-size* *padding* ) x-lenght)))
  ;;(format t "(~3f, ~3f ) ~%" y-lenght x-lenght)
  )


;;---------------------------------------------------
;;-reset the parameter to default
;;---------------------------------------------------
(defun tomato-param-reset ()
  (setf y-lenght default-start)
  (setf x-lenght default-start))



;;---------------------------------------------------
;;tansform start placment position to top-right corner
;;---------------------------------------------------
(defun transform-hako-real-coord ()
  (setf tranform-y (- (elt *hako-center-position* 1) (/ *hako-y-lenght* 2)))
  (setf tranform-x (- (elt *hako-center-position* 0) (/ *hako-x-width* 2)))

  ;;-Add the hasami lenght to the actual hand-coordinate
  ;;(setf tranform-x (- tranform-x *hasami-lenght*))

  ;; values to send to robot to place the tomato
  (if(>= *picking-counter* 1)
      (tomato-arrangement-position))

  (setf *tomato-placing-pos-y* (+ tranform-y y-lenght))
  (setf *tomato-placing-pos-x* (+ tranform-x x-lenght))

  ;; convert this to vector and send
  (setf *tf-vector* (float-vector *tomato-placing-pos-x*
				     *tomato-placing-pos-y*
				     (elt *hako-center-position* 2)))
  (setf *picking-counter* (+ *picking-counter* 1))
  ;;(print *final-tvector*)
  )

;;---------------------------------------------------
;; Call this Function to place the tomato into the hako
;;---------------------------------------------------
(defun hako-arrange-tomato ()
  (transform-hako-real-coord)
  (setf *final-vector* (make-coords :pos *tf-vector*))
  (send *hrp2* :rarm :inverse-kinematics *final-vector*
 	:rotation-axis nil :debug-view t :torso nil)
  )

(defun hako-arrange-tomato-candidate ()
  (transform-hako-real-coord)
  (setf *final-vector* (make-coords :pos *tf-vector*))
  ;; drawing scissors-sized something
  (unless (boundp '*dummy-scissors*)
    (setq *dummy-scissors* (make-cylinder 10 100))
    (send *dummy-scissors* :newcoords
	  (send (send *hrp2* :rarm :end-coords :copy-worldcoords)
		:rotate (deg2rad 90) :y))
    (send (send (send *hrp2* :rarm :end-coords) :parent)
	  :assoc *dummy-scissors*)
    (objects *dummy-scissors*))
  ;; solve IK using coordinates 100mm ahead of HRP2 hand
  (setq *scissors-coords*
	(make-cascoords :coords
			(send *hrp2* :rarm :end-coords :copy-worldcoords)))
  (send *scissors-coords* :translate (float-vector 100 0 0)) ;; preparing new coordinates 100mm ahead of HRP2 hand
  (send (send (send *hrp2* :rarm :end-coords) :parent)
	:assoc *scissors-coords*)
  (send *hrp2* :rarm :inverse-kinematics *final-vector*
	:move-target *scissors-coords*
 	:rotation-axis nil :debug-view t :torso nil)
  )


;;---------------------------------------------------
;; for testing to set robot hand to pick pose
;;---------------------------------------------------
(defun dummpy-pose ()
  (setf r-pose (make-coords :pos (float-vector 690 -260 1050)))
  (send *hrp2* :rarm :inverse-kinematics r-pose
  :rotation-axis nil :debug-view t :torso nil)
)


;;+++++++++++++++++++++++++++++++++++
;; Dummy test function
(defun test(times)
  (dotimes (n times)
    (tomato-arrangement-position))
  )
