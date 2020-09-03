;; Auto-generated. Do not edit!


(when (boundp 'drone::abc)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'abc (find-package "DRONE")))
(unless (find-package "DRONE::ABC")
  (make-package "DRONE::ABC"))

(in-package "ROS")
;;//! \htmlinclude abc.msg.html


(defclass drone::abc
  :super ros::object
  :slots (_data ))

(defmethod drone::abc
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone::abc :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get drone::abc :datatype-) "drone/abc")
(setf (get drone::abc :definition-)
      "float32 data
")



(provide :drone/abc "73fcbf46b49191e672908e50842a83d4")


