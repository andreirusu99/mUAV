; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude Power.msg.html

(cl:defclass <Power> (roslisp-msg-protocol:ros-message)
  ((percentage
    :reader percentage
    :initarg :percentage
    :type cl:float
    :initform 0.0)
   (power
    :reader power
    :initarg :power
    :type cl:float
    :initform 0.0))
)

(cl:defclass Power (<Power>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Power>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Power)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<Power> is deprecated: use drone-msg:Power instead.")))

(cl:ensure-generic-function 'percentage-val :lambda-list '(m))
(cl:defmethod percentage-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:percentage-val is deprecated.  Use drone-msg:percentage instead.")
  (percentage m))

(cl:ensure-generic-function 'power-val :lambda-list '(m))
(cl:defmethod power-val ((m <Power>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:power-val is deprecated.  Use drone-msg:power instead.")
  (power m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Power>) ostream)
  "Serializes a message object of type '<Power>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'percentage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'power))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Power>) istream)
  "Deserializes a message object of type '<Power>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'percentage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'power) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Power>)))
  "Returns string type for a message object of type '<Power>"
  "drone/Power")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Power)))
  "Returns string type for a message object of type 'Power"
  "drone/Power")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Power>)))
  "Returns md5sum for a message object of type '<Power>"
  "5a30afbd53479ee554e4486fc4c1d0de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Power)))
  "Returns md5sum for a message object of type 'Power"
  "5a30afbd53479ee554e4486fc4c1d0de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Power>)))
  "Returns full string definition for message of type '<Power>"
  (cl:format cl:nil "# information about battery and power consumption~%float32 percentage~%float32 power~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Power)))
  "Returns full string definition for message of type 'Power"
  (cl:format cl:nil "# information about battery and power consumption~%float32 percentage~%float32 power~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Power>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Power>))
  "Converts a ROS message object to a list"
  (cl:list 'Power
    (cl:cons ':percentage (percentage msg))
    (cl:cons ':power (power msg))
))
