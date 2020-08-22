;; Auto-generated. Do not edit!


(when (boundp 'drone::Power)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'Power (find-package "DRONE")))
(unless (find-package "DRONE::POWER")
  (make-package "DRONE::POWER"))

(in-package "ROS")
;;//! \htmlinclude Power.msg.html


(defclass drone::Power
  :super ros::object
  :slots (_percentage _power ))

(defmethod drone::Power
  (:init
   (&key
    ((:percentage __percentage) 0.0)
    ((:power __power) 0.0)
    )
   (send-super :init)
   (setq _percentage (float __percentage))
   (setq _power (float __power))
   self)
  (:percentage
   (&optional __percentage)
   (if __percentage (setq _percentage __percentage)) _percentage)
  (:power
   (&optional __power)
   (if __power (setq _power __power)) _power)
  (:serialization-length
   ()
   (+
    ;; float32 _percentage
    4
    ;; float32 _power
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _percentage
       (sys::poke _percentage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _power
       (sys::poke _power (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _percentage
     (setq _percentage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _power
     (setq _power (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone::Power :md5sum-) "5a30afbd53479ee554e4486fc4c1d0de")
(setf (get drone::Power :datatype-) "drone/Power")
(setf (get drone::Power :definition-)
      "# information about battery and power consumption
float32 percentage
float32 power
")



(provide :drone/Power "5a30afbd53479ee554e4486fc4c1d0de")


