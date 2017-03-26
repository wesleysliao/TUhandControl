; Auto-generated. Do not edit!


(cl:in-package stepper_msg-msg)


;//! \htmlinclude Stepper_Target.msg.html

(cl:defclass <Stepper_Target> (roslisp-msg-protocol:ros-message)
  ((position_steps
    :reader position_steps
    :initarg :position_steps
    :type cl:integer
    :initform 0)
   (top_speed_steps_per_second
    :reader top_speed_steps_per_second
    :initarg :top_speed_steps_per_second
    :type cl:integer
    :initform 0))
)

(cl:defclass Stepper_Target (<Stepper_Target>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stepper_Target>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stepper_Target)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stepper_msg-msg:<Stepper_Target> is deprecated: use stepper_msg-msg:Stepper_Target instead.")))

(cl:ensure-generic-function 'position_steps-val :lambda-list '(m))
(cl:defmethod position_steps-val ((m <Stepper_Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stepper_msg-msg:position_steps-val is deprecated.  Use stepper_msg-msg:position_steps instead.")
  (position_steps m))

(cl:ensure-generic-function 'top_speed_steps_per_second-val :lambda-list '(m))
(cl:defmethod top_speed_steps_per_second-val ((m <Stepper_Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stepper_msg-msg:top_speed_steps_per_second-val is deprecated.  Use stepper_msg-msg:top_speed_steps_per_second instead.")
  (top_speed_steps_per_second m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stepper_Target>) ostream)
  "Serializes a message object of type '<Stepper_Target>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'top_speed_steps_per_second)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'top_speed_steps_per_second)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'top_speed_steps_per_second)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'top_speed_steps_per_second)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stepper_Target>) istream)
  "Deserializes a message object of type '<Stepper_Target>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'top_speed_steps_per_second)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'top_speed_steps_per_second)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'top_speed_steps_per_second)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'top_speed_steps_per_second)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stepper_Target>)))
  "Returns string type for a message object of type '<Stepper_Target>"
  "stepper_msg/Stepper_Target")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stepper_Target)))
  "Returns string type for a message object of type 'Stepper_Target"
  "stepper_msg/Stepper_Target")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stepper_Target>)))
  "Returns md5sum for a message object of type '<Stepper_Target>"
  "737f4dc6071f1d903de2da2901e5c3ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stepper_Target)))
  "Returns md5sum for a message object of type 'Stepper_Target"
  "737f4dc6071f1d903de2da2901e5c3ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stepper_Target>)))
  "Returns full string definition for message of type '<Stepper_Target>"
  (cl:format cl:nil "uint32 position_steps~%uint32 top_speed_steps_per_second~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stepper_Target)))
  "Returns full string definition for message of type 'Stepper_Target"
  (cl:format cl:nil "uint32 position_steps~%uint32 top_speed_steps_per_second~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stepper_Target>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stepper_Target>))
  "Converts a ROS message object to a list"
  (cl:list 'Stepper_Target
    (cl:cons ':position_steps (position_steps msg))
    (cl:cons ':top_speed_steps_per_second (top_speed_steps_per_second msg))
))
