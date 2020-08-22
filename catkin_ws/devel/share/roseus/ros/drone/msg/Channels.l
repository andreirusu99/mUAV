;; Auto-generated. Do not edit!


(when (boundp 'drone::Channels)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'Channels (find-package "DRONE")))
(unless (find-package "DRONE::CHANNELS")
  (make-package "DRONE::CHANNELS"))

(in-package "ROS")
;;//! \htmlinclude Channels.msg.html


(defclass drone::Channels
  :super ros::object
  :slots (_channel ))

(defmethod drone::Channels
  (:init
   (&key
    ((:channel __channel) (make-array 6 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _channel __channel)
   self)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:serialization-length
   ()
   (+
    ;; uint16[6] _channel
    (* 2    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[6] _channel
     (dotimes (i 6)
       (write-word (elt _channel i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[6] _channel
   (dotimes (i (length _channel))
     (setf (elt _channel i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get drone::Channels :md5sum-) "31b508c75ec65d5aaa1d5c71970654be")
(setf (get drone::Channels :datatype-) "drone/Channels")
(setf (get drone::Channels :definition-)
      "# represents the channels that the FC accepts
# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2]
uint16[6] channel

")



(provide :drone/Channels "31b508c75ec65d5aaa1d5c71970654be")


