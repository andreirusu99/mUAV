; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude ControlAxes.msg.html

(cl:defclass <ControlAxes> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ControlAxes (<ControlAxes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlAxes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlAxes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<ControlAxes> is deprecated: use drone-msg:ControlAxes instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ControlAxes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:data-val is deprecated.  Use drone-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlAxes>) ostream)
  "Serializes a message object of type '<ControlAxes>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlAxes>) istream)
  "Deserializes a message object of type '<ControlAxes>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlAxes>)))
  "Returns string type for a message object of type '<ControlAxes>"
  "drone/ControlAxes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlAxes)))
  "Returns string type for a message object of type 'ControlAxes"
  "drone/ControlAxes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlAxes>)))
  "Returns md5sum for a message object of type '<ControlAxes>"
  "073fbf0245def3a5b07ad0a112777420")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlAxes)))
  "Returns md5sum for a message object of type 'ControlAxes"
  "073fbf0245def3a5b07ad0a112777420")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlAxes>)))
  "Returns full string definition for message of type '<ControlAxes>"
  (cl:format cl:nil "# represents the channels that the FC accepts~%# [Roll, Pitch, Throttle, Yaw]~%uint16[4] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlAxes)))
  "Returns full string definition for message of type 'ControlAxes"
  (cl:format cl:nil "# represents the channels that the FC accepts~%# [Roll, Pitch, Throttle, Yaw]~%uint16[4] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlAxes>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlAxes>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlAxes
    (cl:cons ':data (data msg))
))
