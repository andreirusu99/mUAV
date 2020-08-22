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
  :slots (_data ))

(defmethod drone::ControlAxes
  (:init
   (&key
    ((:data __data) (make-array 4 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint16[4] _data
    (* 2    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[4] _data
     (dotimes (i 4)
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[4] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get drone::ControlAxes :md5sum-) "073fbf0245def3a5b07ad0a112777420")
(setf (get drone::ControlAxes :datatype-) "drone/ControlAxes")
(setf (get drone::ControlAxes :definition-)
      "# represents the channels that the FC accepts
# [Roll, Pitch, Throttle, Yaw]
uint16[4] data

")



(provide :drone/ControlAxes "073fbf0245def3a5b07ad0a112777420")


