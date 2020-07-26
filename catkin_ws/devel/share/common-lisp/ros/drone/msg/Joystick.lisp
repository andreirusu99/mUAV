; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude Joystick.msg.html

(cl:defclass <Joystick> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Joystick (<Joystick>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Joystick>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Joystick)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<Joystick> is deprecated: use drone-msg:Joystick instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Joystick>) ostream)
  "Serializes a message object of type '<Joystick>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Joystick>) istream)
  "Deserializes a message object of type '<Joystick>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Joystick>)))
  "Returns string type for a message object of type '<Joystick>"
  "drone/Joystick")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Joystick)))
  "Returns string type for a message object of type 'Joystick"
  "drone/Joystick")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Joystick>)))
  "Returns md5sum for a message object of type '<Joystick>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Joystick)))
  "Returns md5sum for a message object of type 'Joystick"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Joystick>)))
  "Returns full string definition for message of type '<Joystick>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Joystick)))
  "Returns full string definition for message of type 'Joystick"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Joystick>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Joystick>))
  "Converts a ROS message object to a list"
  (cl:list 'Joystick
))
