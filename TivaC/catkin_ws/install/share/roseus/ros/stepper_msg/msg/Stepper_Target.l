;; Auto-generated. Do not edit!


(when (boundp 'stepper_msg::Stepper_Target)
  (if (not (find-package "STEPPER_MSG"))
    (make-package "STEPPER_MSG"))
  (shadow 'Stepper_Target (find-package "STEPPER_MSG")))
(unless (find-package "STEPPER_MSG::STEPPER_TARGET")
  (make-package "STEPPER_MSG::STEPPER_TARGET"))

(in-package "ROS")
;;//! \htmlinclude Stepper_Target.msg.html


(defclass stepper_msg::Stepper_Target
  :super ros::object
  :slots (_position_steps _top_speed_steps_per_second ))

(defmethod stepper_msg::Stepper_Target
  (:init
   (&key
    ((:position_steps __position_steps) 0)
    ((:top_speed_steps_per_second __top_speed_steps_per_second) 0)
    )
   (send-super :init)
   (setq _position_steps (round __position_steps))
   (setq _top_speed_steps_per_second (round __top_speed_steps_per_second))
   self)
  (:position_steps
   (&optional __position_steps)
   (if __position_steps (setq _position_steps __position_steps)) _position_steps)
  (:top_speed_steps_per_second
   (&optional __top_speed_steps_per_second)
   (if __top_speed_steps_per_second (setq _top_speed_steps_per_second __top_speed_steps_per_second)) _top_speed_steps_per_second)
  (:serialization-length
   ()
   (+
    ;; uint32 _position_steps
    4
    ;; uint32 _top_speed_steps_per_second
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _position_steps
       (write-long _position_steps s)
     ;; uint32 _top_speed_steps_per_second
       (write-long _top_speed_steps_per_second s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _position_steps
     (setq _position_steps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _top_speed_steps_per_second
     (setq _top_speed_steps_per_second (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get stepper_msg::Stepper_Target :md5sum-) "737f4dc6071f1d903de2da2901e5c3ca")
(setf (get stepper_msg::Stepper_Target :datatype-) "stepper_msg/Stepper_Target")
(setf (get stepper_msg::Stepper_Target :definition-)
      "uint32 position_steps
uint32 top_speed_steps_per_second
")



(provide :stepper_msg/Stepper_Target "737f4dc6071f1d903de2da2901e5c3ca")


