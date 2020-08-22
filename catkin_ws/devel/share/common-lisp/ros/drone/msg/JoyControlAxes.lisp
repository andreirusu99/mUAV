; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude JoyControlAxes.msg.html

(cl:defclass <JoyControlAxes> (roslisp-msg-protocol:ros-message)
  ((axes
    :reader axes
    :initarg :axes
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass JoyControlAxes (<JoyControlAxes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoyControlAxes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoyControlAxes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<JoyControlAxes> is deprecated: use drone-msg:JoyControlAxes instead.")))

(cl:ensure-generic-function 'axes-val :lambda-list '(m))
(cl:defmethod axes-val ((m <JoyControlAxes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:axes-val is deprecated.  Use drone-msg:axes instead.")
  (axes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoyControlAxes>) ostream)
  "Serializes a message object of type '<JoyControlAxes>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'axes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoyControlAxes>) istream)
  "Deserializes a message object of type '<JoyControlAxes>"
  (cl:setf (cl:slot-value msg 'axes) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'axes)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoyControlAxes>)))
  "Returns string type for a message object of type '<JoyControlAxes>"
  "drone/JoyControlAxes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoyControlAxes)))
  "Returns string type for a message object of type 'JoyControlAxes"
  "drone/JoyControlAxes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoyControlAxes>)))
  "Returns md5sum for a message object of type '<JoyControlAxes>"
  "c1b78b75a49603d998b276567ad61bb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoyControlAxes)))
  "Returns md5sum for a message object of type 'JoyControlAxes"
  "c1b78b75a49603d998b276567ad61bb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoyControlAxes>)))
  "Returns full string definition for message of type '<JoyControlAxes>"
  (cl:format cl:nil "# represents the channels that the FC accepts~%# [Roll, Pitch, Throttle, Yaw]~%uint16[4] axes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoyControlAxes)))
  "Returns full string definition for message of type 'JoyControlAxes"
  (cl:format cl:nil "# represents the channels that the FC accepts~%# [Roll, Pitch, Throttle, Yaw]~%uint16[4] axes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoyControlAxes>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'axes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoyControlAxes>))
  "Converts a ROS message object to a list"
  (cl:list 'JoyControlAxes
    (cl:cons ':axes (axes msg))
))
