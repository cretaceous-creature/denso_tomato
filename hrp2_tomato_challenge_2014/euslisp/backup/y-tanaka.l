(load "setup-hrp2-tomato.l")

(defun setup-vmax ()
  (if (not (boundp '*vmax*))
      (progn
        (require :vmax "package://vmax_controller/euslisp/vmax-interface.l")
        (setq *vmax* (instance vmax-interface :init)))))

;; (setup-hrp2-tomato)
;; (reset-tomato-pose)

;; move-base
;; (load "package://vmax_controller/euslisp/vmax_basic_operation.l")
(setup-vmax)
(defun move-base-keyboard
  (&key (vmax *vmax*) (move-time (* 0.1 *base-time*)) (move-dist 100))
  (let (key-input velocity)
    (while t
      (setf key-input (read-line))
      (cond ;; x y z dt
       ((equal key-input "j") (send *vmax* :go-pos-relative-unsafe (- move-dist) 0 0 move-time))
       ((equal key-input "k") (send *vmax* :go-pos-relative-unsafe move-dist 0 0 move-time))
       ((equal key-input "h") (send *vmax* :go-pos-relative-unsafe 0 move-dist 0 move-time))
       ((equal key-input "l") (send *vmax* :go-pos-relative-unsafe 0 (- move-dist)  0 move-time))
       )
      (send *vmax* :wait-interpolation))
    )))

(defun move-base-keyboard-test
  (&key (move-time (* 0.5 *base-time*)) (move-dist 25))
  (let (key-input velocity)
    (while t
      (setf key-input (read-line))
      (cond ;; x y z dt
       ((equal key-input "j") (format t "dx = ~A~%" (- move-dist)));; (send *vmax* :go-pos-relative-unsafe 0 (- move-dist) 0 (move-time)))
       ((equal key-input "k") (format t "dx = ~A~%" move-dist));; (send *vmax* :go-pos-relative-unsafe 0 move-dist 0 (move-time)))
       ((equal key-input "h") (format t "dy = ~A~%" (- move-dist)));; (send *vmax* :go-pos-relative-unsafe (- move-dist) 0 0 (move-time)))
       ((equal key-input "l") (format t "dy = ~A~%" move-dist));; (send *vmax* :go-pos-relative-unsafe move-dist 0 0 (move-time)))
       )
      ))))