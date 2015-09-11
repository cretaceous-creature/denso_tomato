(defclass tomato-model
  :super bodyset
  :slots (fruit pedicel pedicelco))

(defmethod tomato-model
  (:init
   (casco &key (fruit-radius 35) (pedicel-radius 2.5) (pedicel-length 10)
          &allow-other-keys)
   (setq fruit (make-gdome (make-icosahedron fruit-radius)))
   (send fruit :set-color :red)

   (setq pedicel (make-cylinder pedicel-radius pedicel-length))
   (send pedicel :set-color :green)
   (send pedicel :translate (float-vector 0 0 fruit-radius))

   (setq pedicelco
         (make-cascoords
          :pos (float-vector 0 0 (+ fruit-radius pedicel-length))))

   (send fruit :assoc pedicel)
   (send fruit :assoc pedicelco)

   (send fruit :newcoords (send casco :copy-coords))
   (send-super :init casco :bodies (list fruit pedicel))
   self)

  (:pedicel-coords (&rest args) (forward-message-to pedicelco args))
  )


(defun make-tomato (&optional (co (make-cascoords)))
  (instance tomato-model :init co))
