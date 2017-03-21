; Auto-generated. Do not edit!


(cl:in-package adc_joystick_msg-msg)


;//! \htmlinclude ADC_Joystick.msg.html

(cl:defclass <ADC_Joystick> (roslisp-msg-protocol:ros-message)
  ((select
    :reader select
    :initarg :select
    :type cl:boolean
    :initform cl:nil)
   (x_axis_raw
    :reader x_axis_raw
    :initarg :x_axis_raw
    :type cl:fixnum
    :initform 0)
   (y_axis_raw
    :reader y_axis_raw
    :initarg :y_axis_raw
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ADC_Joystick (<ADC_Joystick>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ADC_Joystick>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ADC_Joystick)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name adc_joystick_msg-msg:<ADC_Joystick> is deprecated: use adc_joystick_msg-msg:ADC_Joystick instead.")))

(cl:ensure-generic-function 'select-val :lambda-list '(m))
(cl:defmethod select-val ((m <ADC_Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_joystick_msg-msg:select-val is deprecated.  Use adc_joystick_msg-msg:select instead.")
  (select m))

(cl:ensure-generic-function 'x_axis_raw-val :lambda-list '(m))
(cl:defmethod x_axis_raw-val ((m <ADC_Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_joystick_msg-msg:x_axis_raw-val is deprecated.  Use adc_joystick_msg-msg:x_axis_raw instead.")
  (x_axis_raw m))

(cl:ensure-generic-function 'y_axis_raw-val :lambda-list '(m))
(cl:defmethod y_axis_raw-val ((m <ADC_Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_joystick_msg-msg:y_axis_raw-val is deprecated.  Use adc_joystick_msg-msg:y_axis_raw instead.")
  (y_axis_raw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ADC_Joystick>) ostream)
  "Serializes a message object of type '<ADC_Joystick>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'select) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x_axis_raw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x_axis_raw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y_axis_raw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y_axis_raw)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ADC_Joystick>) istream)
  "Deserializes a message object of type '<ADC_Joystick>"
    (cl:setf (cl:slot-value msg 'select) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x_axis_raw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x_axis_raw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y_axis_raw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y_axis_raw)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ADC_Joystick>)))
  "Returns string type for a message object of type '<ADC_Joystick>"
  "adc_joystick_msg/ADC_Joystick")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ADC_Joystick)))
  "Returns string type for a message object of type 'ADC_Joystick"
  "adc_joystick_msg/ADC_Joystick")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ADC_Joystick>)))
  "Returns md5sum for a message object of type '<ADC_Joystick>"
  "832227e0bdded7eb16fd560e3102fbf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ADC_Joystick)))
  "Returns md5sum for a message object of type 'ADC_Joystick"
  "832227e0bdded7eb16fd560e3102fbf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ADC_Joystick>)))
  "Returns full string definition for message of type '<ADC_Joystick>"
  (cl:format cl:nil "bool select~%uint16 x_axis_raw~%uint16 y_axis_raw~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ADC_Joystick)))
  "Returns full string definition for message of type 'ADC_Joystick"
  (cl:format cl:nil "bool select~%uint16 x_axis_raw~%uint16 y_axis_raw~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ADC_Joystick>))
  (cl:+ 0
     1
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ADC_Joystick>))
  "Converts a ROS message object to a list"
  (cl:list 'ADC_Joystick
    (cl:cons ':select (select msg))
    (cl:cons ':x_axis_raw (x_axis_raw msg))
    (cl:cons ':y_axis_raw (y_axis_raw msg))
))
