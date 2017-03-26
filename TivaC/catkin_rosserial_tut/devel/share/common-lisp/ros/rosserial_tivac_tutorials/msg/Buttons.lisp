; Auto-generated. Do not edit!


(cl:in-package rosserial_tivac_tutorials-msg)


;//! \htmlinclude Buttons.msg.html

(cl:defclass <Buttons> (roslisp-msg-protocol:ros-message)
  ((sw1
    :reader sw1
    :initarg :sw1
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (sw2
    :reader sw2
    :initarg :sw2
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass Buttons (<Buttons>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Buttons>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Buttons)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_tivac_tutorials-msg:<Buttons> is deprecated: use rosserial_tivac_tutorials-msg:Buttons instead.")))

(cl:ensure-generic-function 'sw1-val :lambda-list '(m))
(cl:defmethod sw1-val ((m <Buttons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_tivac_tutorials-msg:sw1-val is deprecated.  Use rosserial_tivac_tutorials-msg:sw1 instead.")
  (sw1 m))

(cl:ensure-generic-function 'sw2-val :lambda-list '(m))
(cl:defmethod sw2-val ((m <Buttons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_tivac_tutorials-msg:sw2-val is deprecated.  Use rosserial_tivac_tutorials-msg:sw2 instead.")
  (sw2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Buttons>) ostream)
  "Serializes a message object of type '<Buttons>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sw1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sw2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Buttons>) istream)
  "Deserializes a message object of type '<Buttons>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sw1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sw2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Buttons>)))
  "Returns string type for a message object of type '<Buttons>"
  "rosserial_tivac_tutorials/Buttons")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Buttons)))
  "Returns string type for a message object of type 'Buttons"
  "rosserial_tivac_tutorials/Buttons")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Buttons>)))
  "Returns md5sum for a message object of type '<Buttons>"
  "a78ccaade8fa723d1ebeb7b099042085")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Buttons)))
  "Returns md5sum for a message object of type 'Buttons"
  "a78ccaade8fa723d1ebeb7b099042085")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Buttons>)))
  "Returns full string definition for message of type '<Buttons>"
  (cl:format cl:nil "std_msgs/Bool sw1~%std_msgs/Bool sw2~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Buttons)))
  "Returns full string definition for message of type 'Buttons"
  (cl:format cl:nil "std_msgs/Bool sw1~%std_msgs/Bool sw2~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Buttons>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sw1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sw2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Buttons>))
  "Converts a ROS message object to a list"
  (cl:list 'Buttons
    (cl:cons ':sw1 (sw1 msg))
    (cl:cons ':sw2 (sw2 msg))
))
