; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude SonarN.msg.html

(cl:defclass <SonarN> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass SonarN (<SonarN>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SonarN>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SonarN)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<SonarN> is deprecated: use drone-msg:SonarN instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <SonarN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:distance-val is deprecated.  Use drone-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SonarN>) ostream)
  "Serializes a message object of type '<SonarN>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SonarN>) istream)
  "Deserializes a message object of type '<SonarN>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SonarN>)))
  "Returns string type for a message object of type '<SonarN>"
  "drone/SonarN")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SonarN)))
  "Returns string type for a message object of type 'SonarN"
  "drone/SonarN")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SonarN>)))
  "Returns md5sum for a message object of type '<SonarN>"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SonarN)))
  "Returns md5sum for a message object of type 'SonarN"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SonarN>)))
  "Returns full string definition for message of type '<SonarN>"
  (cl:format cl:nil "# the distance between the sensor and the ground~%float32 distance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SonarN)))
  "Returns full string definition for message of type 'SonarN"
  (cl:format cl:nil "# the distance between the sensor and the ground~%float32 distance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SonarN>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SonarN>))
  "Converts a ROS message object to a list"
  (cl:list 'SonarN
    (cl:cons ':distance (distance msg))
))
