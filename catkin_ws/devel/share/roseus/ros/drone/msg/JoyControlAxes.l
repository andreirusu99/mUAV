;; Auto-generated. Do not edit!


(when (boundp 'drone::JoyControlAxes)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'JoyControlAxes (find-package "DRONE")))
(unless (find-package "DRONE::JOYCONTROLAXES")
  (make-package "DRONE::JOYCONTROLAXES"))

(in-package "ROS")
;;//! \htmlinclude JoyControlAxes.msg.html


(defclass drone::JoyControlAxes
  :super ros::object
  :slots (_axes ))

(defmethod drone::JoyControlAxes
  (:init
   (&key
    ((:axes __axes) (make-array 4 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _axes __axes)
   self)
  (:axes
   (&optional __axes)
   (if __axes (setq _axes __axes)) _axes)
  (:serialization-length
   ()
   (+
    ;; uint16[4] _axes
    (* 2    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[4] _axes
     (dotimes (i 4)
       (write-word (elt _axes i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[4] _axes
   (dotimes (i (length _axes))
     (setf (elt _axes i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get drone::JoyControlAxes :md5sum-) "c1b78b75a49603d998b276567ad61bb1")
(setf (get drone::JoyControlAxes :datatype-) "drone/JoyControlAxes")
(setf (get drone::JoyControlAxes :definition-)
      "# represents the channels that the FC accepts
# [Roll, Pitch, Throttle, Yaw]
uint16[4] axes

")



(provide :drone/JoyControlAxes "c1b78b75a49603d998b276567ad61bb1")


