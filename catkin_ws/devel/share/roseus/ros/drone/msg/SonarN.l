;; Auto-generated. Do not edit!


(when (boundp 'drone::SonarN)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'SonarN (find-package "DRONE")))
(unless (find-package "DRONE::SONARN")
  (make-package "DRONE::SONARN"))

(in-package "ROS")
;;//! \htmlinclude SonarN.msg.html


(defclass drone::SonarN
  :super ros::object
  :slots (_distance ))

(defmethod drone::SonarN
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

(setf (get drone::SonarN :md5sum-) "6e77fb10f0c8b4833ec273aa9ac74459")
(setf (get drone::SonarN :datatype-) "drone/SonarN")
(setf (get drone::SonarN :definition-)
      "# the distance between the sensor and the ground
float32 distance
")



(provide :drone/SonarN "6e77fb10f0c8b4833ec273aa9ac74459")


