(load "package://aria_model/euslisp/aria-interface.l")
(load "package://c2_tomato_challenge_2014/euslisp/tomato-motion-lib.l")
(load "package://c2_tomato_challenge_2014/euslisp/c2-util.l")

(defvar *real* t)
(defvar *base-time* 1500)

(defun setup-c2-tomato ()
  (unless *aria*
    (aria-init)
    (c2-util-init))
  ;; draw hako
  (unless (boundp '*hako*)
    (let (hako-all hako-off)
      (setq hako-all (make-cube 260 350 80))
      ;; the origin point is located in the center
      (setq hako-off (make-cube 250 340 80))
      (send hako-off :locate #f(0 0 10))
      (setq *hako* (body- hako-all hako-off))
      ;; (send *hako* :locate #f(250 0 -80))
      (send *hako* :locate #f(0 -250 -80))
      (send *hako* :rotate (deg2rad 90) :z)
      (send *hako* :set-color :green)
      (send *aria* :assoc *hako*)
      ))
  ;;12.13
  (objects (list *aria* *hako*))
  (real2model)
  )
(setup-c2-tomato)
