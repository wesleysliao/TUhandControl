;; Auto-generated. Do not edit!


(when (boundp 'rosserial_tivac_tutorials::Buttons)
  (if (not (find-package "ROSSERIAL_TIVAC_TUTORIALS"))
    (make-package "ROSSERIAL_TIVAC_TUTORIALS"))
  (shadow 'Buttons (find-package "ROSSERIAL_TIVAC_TUTORIALS")))
(unless (find-package "ROSSERIAL_TIVAC_TUTORIALS::BUTTONS")
  (make-package "ROSSERIAL_TIVAC_TUTORIALS::BUTTONS"))

(in-package "ROS")
;;//! \htmlinclude Buttons.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rosserial_tivac_tutorials::Buttons
  :super ros::object
  :slots (_sw1 _sw2 ))

(defmethod rosserial_tivac_tutorials::Buttons
  (:init
   (&key
    ((:sw1 __sw1) (instance std_msgs::Bool :init))
    ((:sw2 __sw2) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _sw1 __sw1)
   (setq _sw2 __sw2)
   self)
  (:sw1
   (&rest __sw1)
   (if (keywordp (car __sw1))
       (send* _sw1 __sw1)
     (progn
       (if __sw1 (setq _sw1 (car __sw1)))
       _sw1)))
  (:sw2
   (&rest __sw2)
   (if (keywordp (car __sw2))
       (send* _sw2 __sw2)
     (progn
       (if __sw2 (setq _sw2 (car __sw2)))
       _sw2)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Bool _sw1
    (send _sw1 :serialization-length)
    ;; std_msgs/Bool _sw2
    (send _sw2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Bool _sw1
       (send _sw1 :serialize s)
     ;; std_msgs/Bool _sw2
       (send _sw2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Bool _sw1
     (send _sw1 :deserialize buf ptr-) (incf ptr- (send _sw1 :serialization-length))
   ;; std_msgs/Bool _sw2
     (send _sw2 :deserialize buf ptr-) (incf ptr- (send _sw2 :serialization-length))
   ;;
   self)
  )

(setf (get rosserial_tivac_tutorials::Buttons :md5sum-) "a78ccaade8fa723d1ebeb7b099042085")
(setf (get rosserial_tivac_tutorials::Buttons :datatype-) "rosserial_tivac_tutorials/Buttons")
(setf (get rosserial_tivac_tutorials::Buttons :definition-)
      "std_msgs/Bool sw1
std_msgs/Bool sw2

================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :rosserial_tivac_tutorials/Buttons "a78ccaade8fa723d1ebeb7b099042085")


