; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude Channels.msg.html

(cl:defclass <Channels> (roslisp-msg-protocol:ros-message)
  ((channel
    :reader channel
    :initarg :channel
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Channels (<Channels>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Channels>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Channels)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<Channels> is deprecated: use drone-msg:Channels instead.")))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <Channels>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:channel-val is deprecated.  Use drone-msg:channel instead.")
  (channel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Channels>) ostream)
  "Serializes a message object of type '<Channels>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'channel))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Channels>) istream)
  "Deserializes a message object of type '<Channels>"
  (cl:setf (cl:slot-value msg 'channel) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'channel)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Channels>)))
  "Returns string type for a message object of type '<Channels>"
  "drone/Channels")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Channels)))
  "Returns string type for a message object of type 'Channels"
  "drone/Channels")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Channels>)))
  "Returns md5sum for a message object of type '<Channels>"
  "31b508c75ec65d5aaa1d5c71970654be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Channels)))
  "Returns md5sum for a message object of type 'Channels"
  "31b508c75ec65d5aaa1d5c71970654be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Channels>)))
  "Returns full string definition for message of type '<Channels>"
  (cl:format cl:nil "# represents the channels that the FC accepts~%# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2]~%uint16[6] channel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Channels)))
  "Returns full string definition for message of type 'Channels"
  (cl:format cl:nil "# represents the channels that the FC accepts~%# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2]~%uint16[6] channel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Channels>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'channel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Channels>))
  "Converts a ROS message object to a list"
  (cl:list 'Channels
    (cl:cons ':channel (channel msg))
))
