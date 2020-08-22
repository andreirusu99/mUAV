;; Auto-generated. Do not edit!


(when (boundp 'drone::Joystick)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'Joystick (find-package "DRONE")))
(unless (find-package "DRONE::JOYSTICK")
  (make-package "DRONE::JOYSTICK"))

(in-package "ROS")
;;//! \htmlinclude Joystick.msg.html


(defclass drone::Joystick
  :super ros::object
  :slots ())

(defmethod drone::Joystick
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get drone::Joystick :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get drone::Joystick :datatype-) "drone/Joystick")
(setf (get drone::Joystick :definition-)
      "
")



(provide :drone/Joystick "d41d8cd98f00b204e9800998ecf8427e")


