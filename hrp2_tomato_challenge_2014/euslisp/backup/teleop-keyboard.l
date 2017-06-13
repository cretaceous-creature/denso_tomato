(defun move-base-keyboard
  (&key (vmax *vmax*) (move-time (* 0.5 *base-time*)) (move-dist 25))
  (let (key-input velocity)
    (while t
      (setf key-input (read-line))
      (cond ;; x y z dt
       ((equal key-input "j") (send *vmax* :go-pos-relative-unsafe 0 (- move-dist) 0 (move-time)))
       ((equal key-input "k") (send *vmax* :go-pos-relative-unsafe 0 move-dist 0 (move-time)))
       ((equal key-input "h") (send *vmax* :go-pos-relative-unsafe (- move-dist) 0 0 (move-time)))
       ((equal key-input "l") (send *vmax* :go-pos-relative-unsafe move-dist 0 0 (move-time)))
       )
      (send *vmax* :wait-interpolaton))
    )))