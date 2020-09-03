;; Auto-generated. Do not edit!


(when (boundp 'drone::Sonar)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'Sonar (find-package "DRONE")))
(unless (find-package "DRONE::SONAR")
  (make-package "DRONE::SONAR"))

(in-package "ROS")
;;//! \htmlinclude Sonar.msg.html


(defclass drone::Sonar
  :super ros::object
  :slots (_distance ))

(defmethod drone::Sonar
  (:init
   (&key
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone::Sonar :md5sum-) "6e77fb10f0c8b4833ec273aa9ac74459")
(setf (get drone::Sonar :datatype-) "drone/Sonar")
(setf (get drone::Sonar :definition-)
      "# the distance between the sensor and the ground
float32 distance
")



(provide :drone/Sonar "6e77fb10f0c8b4833ec273aa9ac74459")


