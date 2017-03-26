; Auto-generated. Do not edit!


(cl:in-package rosserial_tivac_tutorials-srv)


;//! \htmlinclude ColorRGBA-request.msg.html

(cl:defclass <ColorRGBA-request> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA)))
)

(cl:defclass ColorRGBA-request (<ColorRGBA-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColorRGBA-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColorRGBA-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_tivac_tutorials-srv:<ColorRGBA-request> is deprecated: use rosserial_tivac_tutorials-srv:ColorRGBA-request instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <ColorRGBA-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_tivac_tutorials-srv:color-val is deprecated.  Use rosserial_tivac_tutorials-srv:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColorRGBA-request>) ostream)
  "Serializes a message object of type '<ColorRGBA-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'color) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColorRGBA-request>) istream)
  "Deserializes a message object of type '<ColorRGBA-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'color) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColorRGBA-request>)))
  "Returns string type for a service object of type '<ColorRGBA-request>"
  "rosserial_tivac_tutorials/ColorRGBARequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColorRGBA-request)))
  "Returns string type for a service object of type 'ColorRGBA-request"
  "rosserial_tivac_tutorials/ColorRGBARequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColorRGBA-request>)))
  "Returns md5sum for a message object of type '<ColorRGBA-request>"
  "f0106025071e27ff96990e664f228cc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColorRGBA-request)))
  "Returns md5sum for a message object of type 'ColorRGBA-request"
  "f0106025071e27ff96990e664f228cc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColorRGBA-request>)))
  "Returns full string definition for message of type '<ColorRGBA-request>"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColorRGBA-request)))
  "Returns full string definition for message of type 'ColorRGBA-request"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColorRGBA-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColorRGBA-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ColorRGBA-request
    (cl:cons ':color (color msg))
))
;//! \htmlinclude ColorRGBA-response.msg.html

(cl:defclass <ColorRGBA-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ColorRGBA-response (<ColorRGBA-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColorRGBA-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColorRGBA-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_tivac_tutorials-srv:<ColorRGBA-response> is deprecated: use rosserial_tivac_tutorials-srv:ColorRGBA-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ColorRGBA-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_tivac_tutorials-srv:result-val is deprecated.  Use rosserial_tivac_tutorials-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColorRGBA-response>) ostream)
  "Serializes a message object of type '<ColorRGBA-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColorRGBA-response>) istream)
  "Deserializes a message object of type '<ColorRGBA-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColorRGBA-response>)))
  "Returns string type for a service object of type '<ColorRGBA-response>"
  "rosserial_tivac_tutorials/ColorRGBAResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColorRGBA-response)))
  "Returns string type for a service object of type 'ColorRGBA-response"
  "rosserial_tivac_tutorials/ColorRGBAResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColorRGBA-response>)))
  "Returns md5sum for a message object of type '<ColorRGBA-response>"
  "f0106025071e27ff96990e664f228cc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColorRGBA-response)))
  "Returns md5sum for a message object of type 'ColorRGBA-response"
  "f0106025071e27ff96990e664f228cc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColorRGBA-response>)))
  "Returns full string definition for message of type '<ColorRGBA-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColorRGBA-response)))
  "Returns full string definition for message of type 'ColorRGBA-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColorRGBA-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColorRGBA-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ColorRGBA-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ColorRGBA)))
  'ColorRGBA-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ColorRGBA)))
  'ColorRGBA-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColorRGBA)))
  "Returns string type for a service object of type '<ColorRGBA>"
  "rosserial_tivac_tutorials/ColorRGBA")