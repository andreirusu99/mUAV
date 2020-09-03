; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude Smth.msg.html

(cl:defclass <Smth> (roslisp-msg-protocol:ros-message)
  ((ground_distance
    :reader ground_distance
    :initarg :ground_distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass Smth (<Smth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Smth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Smth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<Smth> is deprecated: use drone-msg:Smth instead.")))

(cl:ensure-generic-function 'ground_distance-val :lambda-list '(m))
(cl:defmethod ground_distance-val ((m <Smth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:ground_distance-val is deprecated.  Use drone-msg:ground_distance instead.")
  (ground_distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Smth>) ostream)
  "Serializes a message object of type '<Smth>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ground_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Smth>) istream)
  "Deserializes a message object of type '<Smth>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ground_distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Smth>)))
  "Returns string type for a message object of type '<Smth>"
  "drone/Smth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Smth)))
  "Returns string type for a message object of type 'Smth"
  "drone/Smth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Smth>)))
  "Returns md5sum for a message object of type '<Smth>"
  "8f146d4e34a63c94036de552a9e31ac8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Smth)))
  "Returns md5sum for a message object of type 'Smth"
  "8f146d4e34a63c94036de552a9e31ac8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Smth>)))
  "Returns full string definition for message of type '<Smth>"
  (cl:format cl:nil "# from sonar~%float32 ground_distance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Smth)))
  "Returns full string definition for message of type 'Smth"
  (cl:format cl:nil "# from sonar~%float32 ground_distance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Smth>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Smth>))
  "Converts a ROS message object to a list"
  (cl:list 'Smth
    (cl:cons ':ground_distance (ground_distance msg))
))
