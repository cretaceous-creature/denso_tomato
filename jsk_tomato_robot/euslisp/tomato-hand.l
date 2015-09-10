(defun make-tomato-hand ()
  (let (craw0 craw1 craw2 suction hand-body)
    (setq craw0 (make-cube 80 40 10)
          craw1 (make-cube 80 40 10)
          craw2 (make-cube 80 40 10))
    (send craw0 :set-color :white)
    (send craw1 :set-color :white)
    (send craw2 :set-color :white)

    (send craw1 :rotate (deg2rad 120) :x)
    (send craw2 :rotate (deg2rad -120) :x)

    (send craw0 :translate (float-vector 70 0 25))
    (send craw1 :translate (float-vector 70 0 25))
    (send craw2 :translate (float-vector 70 0 25))

    (setq suction (make-cylinder 20 70)
          hand-body (make-cylinder 40 10))
    (send suction :set-color :white)
    (send hand-body :set-color :white)

    (send suction :rotate pi/2 :y)
    (send hand-body :rotate pi/2 :y)

    (send hand-body :assoc craw0)
    (send hand-body :assoc craw1)
    (send hand-body :assoc craw2)
    (send hand-body :assoc suction)

    (instance bodyset :init
              (make-cascoords)
              :bodies (list hand-body suction craw0 craw1 craw2))

    )
  )