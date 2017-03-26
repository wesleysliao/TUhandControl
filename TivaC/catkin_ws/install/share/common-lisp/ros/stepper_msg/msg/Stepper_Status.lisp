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
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stepper_Status>) ostream)
  "Serializes a message object of type '<Stepper_Status>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_steps_per_second)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed_steps_per_second)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'speed_steps_per_second)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'speed_steps_per_second)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'direction_forward) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stepper_Status>) istream)
  "Deserializes a message object of type '<Stepper_Status>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_steps_per_second)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed_steps_per_second)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'speed_steps_per_second)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'speed_steps_per_second)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'direction_forward) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "3a08b832803b195f0f005fead32aedac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stepper_Status)))
  "Returns md5sum for a message object of type 'Stepper_Status"
  "3a08b832803b195f0f005fead32aedac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stepper_Status>)))
  "Returns full string definition for message of type '<Stepper_Status>"
  (cl:format cl:nil "uint32 position_steps~%uint32 speed_steps_per_second~%bool direction_forward~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stepper_Status)))
  "Returns full string definition for message of type 'Stepper_Status"
  (cl:format cl:nil "uint32 position_steps~%uint32 speed_steps_per_second~%bool direction_forward~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stepper_Status>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stepper_Status>))
  "Converts a ROS message object to a list"
  (cl:list 'Stepper_Status
    (cl:cons ':position_steps (position_steps msg))
    (cl:cons ':speed_steps_per_second (speed_steps_per_second msg))
    (cl:cons ':direction_forward (direction_forward msg))
))
