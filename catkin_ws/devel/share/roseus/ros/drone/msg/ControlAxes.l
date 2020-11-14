;; Auto-generated. Do not edit!


(when (boundp 'drone::ControlAxes)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'ControlAxes (find-package "DRONE")))
(unless (find-package "DRONE::CONTROLAXES")
  (make-package "DRONE::CONTROLAXES"))

(in-package "ROS")
;;//! \htmlinclude ControlAxes.msg.html


(defclass drone::ControlAxes
  :super ros::object
  :slots (_axis ))

(defmethod drone::ControlAxes
  (:init
   (&key
    ((:axis __axis) (make-array 4 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _axis __axis)
   self)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:serialization-length
   ()
   (+
    ;; uint16[4] _axis
    (* 2    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[4] _axis
     (dotimes (i 4)
       (write-word (elt _axis i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[4] _axis
   (dotimes (i (length _axis))
     (setf (elt _axis i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get drone::ControlAxes :md5sum-) "4c8bf3d9704c3cb6fc3e2299ec7776bb")
(setf (get drone::ControlAxes :datatype-) "drone/ControlAxes")
(setf (get drone::ControlAxes :definition-)
      "# represents the channels that the FC accepts
# [Roll, Pitch, Throttle, Yaw]
uint16[4] axis

")



(provide :drone/ControlAxes "4c8bf3d9704c3cb6fc3e2299ec7776bb")


