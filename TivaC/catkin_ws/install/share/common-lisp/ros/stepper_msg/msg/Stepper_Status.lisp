; Auto-generated. Do not edit!


(cl:in-package stepper_msg-msg)


;//! \htmlinclude Stepper_Status.msg.html

(cl:defclass <Stepper_Status> (roslisp-msg-protocol:ros-message)
  ((position_steps
    :reader position_steps
    :initarg :position_steps
    :type cl:integer
    :initform 0)
   (speed_steps_per_second
    :reader speed_steps_per_second
    :initarg :speed_steps_per_second
    :type cl:integer
    :initform 0)
   (direction_forward
    :reader direction_forward
    :initarg :direction_forward
    :type cl:boolean
    :initform cl:nil)
   (enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Stepper_Status (<Stepper_Status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stepper_Status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stepper_Status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stepper_msg-msg:<Stepper_Status> is deprecated: use stepper_msg-msg:Stepper_Status instead.")))

(cl:ensure-generic-function 'position_steps-val :lambda-list '(m))
(cl:defmethod position_steps-val ((m <Stepper_Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stepper_msg-msg:position_steps-val is deprecated.  Use stepper_msg-msg:position_steps instead.")
  (position_steps m))

(cl:ensure-generic-function 'speed_steps_per_second-val :lambda-list '(m))
(cl:defmethod speed_steps_per_second-val ((m <Stepper_Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stepper_msg-msg:speed_steps_per_second-val is deprecated.  Use stepper_msg-msg:speed_steps_per_second instead.")
  (speed_steps_per_second m))

(cl:ensure-generic-function 'direction_forward-val :lambda-list '(m))
(cl:defmethod direction_forward-val ((m <Stepper_Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stepper_msg-msg:direction_forward-val is deprecated.  Use stepper_msg-msg:direction_forward instead.")
  (direction_forward m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <Stepper_Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stepper_msg-msg:enabled-val is deprecated.  Use stepper_msg-msg:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stepper_Status>) ostream)
  "Serializes a message object of type '<Stepper_Status>"
  (cl:let* ((signed (cl:slot-value msg 'position_steps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed_steps_per_second)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'direction_forward) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stepper_Status>) istream)
  "Deserializes a message object of type '<Stepper_Status>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position_steps) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed_steps_per_second) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'direction_forward) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stepper_Status>)))
  "Returns string type for a message object of type '<Stepper_Status>"
  "stepper_msg/Stepper_Status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stepper_Status)))
  "Returns string type for a message object of type 'Stepper_Status"
  "stepper_msg/Stepper_Status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stepper_Status>)))
  "Returns md5sum for a message object of type '<Stepper_Status>"
  "b8e41235ddba5043cc88e78b8401e13b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stepper_Status)))
  "Returns md5sum for a message object of type 'Stepper_Status"
  "b8e41235ddba5043cc88e78b8401e13b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stepper_Status>)))
  "Returns full string definition for message of type '<Stepper_Status>"
  (cl:format cl:nil "int64 position_steps~%int32 speed_steps_per_second~%bool direction_forward~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stepper_Status)))
  "Returns full string definition for message of type 'Stepper_Status"
  (cl:format cl:nil "int64 position_steps~%int32 speed_steps_per_second~%bool direction_forward~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stepper_Status>))
  (cl:+ 0
     8
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stepper_Status>))
  "Converts a ROS message object to a list"
  (cl:list 'Stepper_Status
    (cl:cons ':position_steps (position_steps msg))
    (cl:cons ':speed_steps_per_second (speed_steps_per_second msg))
    (cl:cons ':direction_forward (direction_forward msg))
    (cl:cons ':enabled (enabled msg))
))
