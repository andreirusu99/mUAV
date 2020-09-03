;; Auto-generated. Do not edit!


(when (boundp 'drone::SonarAltitude)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'SonarAltitude (find-package "DRONE")))
(unless (find-package "DRONE::SONARALTITUDE")
  (make-package "DRONE::SONARALTITUDE"))

(in-package "ROS")
;;//! \htmlinclude SonarAltitude.msg.html


(defclass drone::SonarAltitude
  :super ros::object
  :slots (_ground_distance ))

(defmethod drone::SonarAltitude
  (:init
   (&key
    ((:ground_distance __ground_distance) 0.0)
    )
   (send-super :init)
   (setq _ground_distance (float __ground_distance))
   self)
  (:ground_distance
   (&optional __ground_distance)
   (if __ground_distance (setq _ground_distance __ground_distance)) _ground_distance)
  (:serialization-length
   ()
   (+
    ;; float32 _ground_distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _ground_distance
       (sys::poke _ground_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _ground_distance
     (setq _ground_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone::SonarAltitude :md5sum-) "8f146d4e34a63c94036de552a9e31ac8")
(setf (get drone::SonarAltitude :datatype-) "drone/SonarAltitude")
(setf (get drone::SonarAltitude :definition-)
      "# from sonar
float32 ground_distance
")



(provide :drone/SonarAltitude "8f146d4e34a63c94036de552a9e31ac8")


