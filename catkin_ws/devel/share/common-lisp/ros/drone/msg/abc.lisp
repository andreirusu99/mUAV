; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude abc.msg.html

(cl:defclass <abc> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass abc (<abc>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <abc>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'abc)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<abc> is deprecated: use drone-msg:abc instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <abc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:data-val is deprecated.  Use drone-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <abc>) ostream)
  "Serializes a message object of type '<abc>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <abc>) istream)
  "Deserializes a message object of type '<abc>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<abc>)))
  "Returns string type for a message object of type '<abc>"
  "drone/abc")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'abc)))
  "Returns string type for a message object of type 'abc"
  "drone/abc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<abc>)))
  "Returns md5sum for a message object of type '<abc>"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'abc)))
  "Returns md5sum for a message object of type 'abc"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<abc>)))
  "Returns full string definition for message of type '<abc>"
  (cl:format cl:nil "float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'abc)))
  "Returns full string definition for message of type 'abc"
  (cl:format cl:nil "float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <abc>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <abc>))
  "Converts a ROS message object to a list"
  (cl:list 'abc
    (cl:cons ':data (data msg))
))
