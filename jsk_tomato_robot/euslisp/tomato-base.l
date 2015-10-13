(defun make-tomato-base (&key (base-height 500))
  (let (base-body hako hako-all hako-off)
    (setq base-body (make-cube 800 600 base-height))
    (send base-body :set-color :gray)
    (send base-body :translate (float-vector 0 0 (* 0.5 base-height)))

    ;; from setup-hrp2-tomato.l
    (setq hako-all (make-cube 250 350 100)) ;;the origin point is located in the center
    (setq hako-off (make-cube 220 330 70))
    (send hako-off :locate #f(0 0 30))
    (setq hako (body- hako-all hako-off))

    (send hako :set-color :brown)
    (send hako :translate (float-vector 200 0 (+ base-height 50)))

    (instance bodyset :init
              (make-cascoords)
              :bodies (list base-body hako))
    )
  )
