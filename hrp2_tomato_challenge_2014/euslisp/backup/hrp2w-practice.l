(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
(hrp2w-init)
(objects *hrp2w*)
(send *hrp2w* :reset-manip-pose)
(send *ri* :angle-vector (send *hrp2w* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;(send *hrp2w* :rarm :end-coords)
;(setq *co* (send * :copy-worldcoords))
(setq *co* (send (send *hrp2w* :rarm :end-coords) 
:copy-worldcoords))
(send *co* :translate (float-vector 100 0 0))
(send *hrp2w* :rarm :inverse-kinematics *co*)
(send *hrp2w* :head :look-at (send *hrp2w* :rarm :end-coords :pos))
(send *hrp2w* :head :neck-p :joint-angle 0.0)
(send *irtviewer* :draw-objects)