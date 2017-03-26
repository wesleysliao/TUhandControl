;; Auto-generated. Do not edit!


(when (boundp 'rosserial_tivac_tutorials::ColorRGBA)
  (if (not (find-package "ROSSERIAL_TIVAC_TUTORIALS"))
    (make-package "ROSSERIAL_TIVAC_TUTORIALS"))
  (shadow 'ColorRGBA (find-package "ROSSERIAL_TIVAC_TUTORIALS")))
(unless (find-package "ROSSERIAL_TIVAC_TUTORIALS::COLORRGBA")
  (make-package "ROSSERIAL_TIVAC_TUTORIALS::COLORRGBA"))
(unless (find-package "ROSSERIAL_TIVAC_TUTORIALS::COLORRGBAREQUEST")
  (make-package "ROSSERIAL_TIVAC_TUTORIALS::COLORRGBAREQUEST"))
(unless (find-package "ROSSERIAL_TIVAC_TUTORIALS::COLORRGBARESPONSE")
  (make-package "ROSSERIAL_TIVAC_TUTORIALS::COLORRGBARESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass rosserial_tivac_tutorials::ColorRGBARequest
  :super ros::object
  :slots (_color ))

(defmethod rosserial_tivac_tutorials::ColorRGBARequest
  (:init
   (&key
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    )
   (send-super :init)
   (setq _color __color)
   self)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;;
   self)
  )

(defclass rosserial_tivac_tutorials::ColorRGBAResponse
  :super ros::object
  :slots (_result ))

(defmethod rosserial_tivac_tutorials::ColorRGBAResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rosserial_tivac_tutorials::ColorRGBA
  :super ros::object
  :slots ())

(setf (get rosserial_tivac_tutorials::ColorRGBA :md5sum-) "f0106025071e27ff96990e664f228cc8")
(setf (get rosserial_tivac_tutorials::ColorRGBA :datatype-) "rosserial_tivac_tutorials/ColorRGBA")
(setf (get rosserial_tivac_tutorials::ColorRGBA :request) rosserial_tivac_tutorials::ColorRGBARequest)
(setf (get rosserial_tivac_tutorials::ColorRGBA :response) rosserial_tivac_tutorials::ColorRGBAResponse)

(defmethod rosserial_tivac_tutorials::ColorRGBARequest
  (:response () (instance rosserial_tivac_tutorials::ColorRGBAResponse :init)))

(setf (get rosserial_tivac_tutorials::ColorRGBARequest :md5sum-) "f0106025071e27ff96990e664f228cc8")
(setf (get rosserial_tivac_tutorials::ColorRGBARequest :datatype-) "rosserial_tivac_tutorials/ColorRGBARequest")
(setf (get rosserial_tivac_tutorials::ColorRGBARequest :definition-)
      "std_msgs/ColorRGBA color

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
---
bool result

")

(setf (get rosserial_tivac_tutorials::ColorRGBAResponse :md5sum-) "f0106025071e27ff96990e664f228cc8")
(setf (get rosserial_tivac_tutorials::ColorRGBAResponse :datatype-) "rosserial_tivac_tutorials/ColorRGBAResponse")
(setf (get rosserial_tivac_tutorials::ColorRGBAResponse :definition-)
      "std_msgs/ColorRGBA color

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
---
bool result

")



(provide :rosserial_tivac_tutorials/ColorRGBA "f0106025071e27ff96990e664f228cc8")


