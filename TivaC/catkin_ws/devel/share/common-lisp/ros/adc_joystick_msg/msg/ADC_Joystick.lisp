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
   (x_axis_zero
    :reader x_axis_zero
    :initarg :x_axis_zero
    :type cl:fixnum
    :initform 0)
   (y_axis_raw
    :reader y_axis_raw
    :initarg :y_axis_raw
    :type cl:fixnum
    :initform 0)
   (y_axis_zero
    :reader y_axis_zero
    :initarg :y_axis_zero
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

(cl:ensure-generic-function 'x_axis_zero-val :lambda-list '(m))
(cl:defmethod x_axis_zero-val ((m <ADC_Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_joystick_msg-msg:x_axis_zero-val is deprecated.  Use adc_joystick_msg-msg:x_axis_zero instead.")
  (x_axis_zero m))

(cl:ensure-generic-function 'y_axis_raw-val :lambda-list '(m))
(cl:defmethod y_axis_raw-val ((m <ADC_Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_joystick_msg-msg:y_axis_raw-val is deprecated.  Use adc_joystick_msg-msg:y_axis_raw instead.")
  (y_axis_raw m))

(cl:ensure-generic-function 'y_axis_zero-val :lambda-list '(m))
(cl:defmethod y_axis_zero-val ((m <ADC_Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_joystick_msg-msg:y_axis_zero-val is deprecated.  Use adc_joystick_msg-msg:y_axis_zero instead.")
  (y_axis_zero m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ADC_Joystick>) ostream)
  "Serializes a message object of type '<ADC_Joystick>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'select) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'x_axis_raw)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x_axis_zero)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_axis_raw)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_axis_zero)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ADC_Joystick>) istream)
  "Deserializes a message object of type '<ADC_Joystick>"
    (cl:setf (cl:slot-value msg 'select) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_axis_raw) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_axis_zero) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_axis_raw) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_axis_zero) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
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
  "10957cbdb8481676f0a00f626dbd9899")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ADC_Joystick)))
  "Returns md5sum for a message object of type 'ADC_Joystick"
  "10957cbdb8481676f0a00f626dbd9899")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ADC_Joystick>)))
  "Returns full string definition for message of type '<ADC_Joystick>"
  (cl:format cl:nil "bool select~%int16 x_axis_raw~%int16 x_axis_zero~%int16 y_axis_raw~%int16 y_axis_zero~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ADC_Joystick)))
  "Returns full string definition for message of type 'ADC_Joystick"
  (cl:format cl:nil "bool select~%int16 x_axis_raw~%int16 x_axis_zero~%int16 y_axis_raw~%int16 y_axis_zero~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ADC_Joystick>))
  (cl:+ 0
     1
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ADC_Joystick>))
  "Converts a ROS message object to a list"
  (cl:list 'ADC_Joystick
    (cl:cons ':select (select msg))
    (cl:cons ':x_axis_raw (x_axis_raw msg))
    (cl:cons ':x_axis_zero (x_axis_zero msg))
    (cl:cons ':y_axis_raw (y_axis_raw msg))
    (cl:cons ':y_axis_zero (y_axis_zero msg))
))
