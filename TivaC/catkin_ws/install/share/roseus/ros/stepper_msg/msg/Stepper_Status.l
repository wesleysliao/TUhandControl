;; Auto-generated. Do not edit!


(when (boundp 'stepper_msg::Stepper_Status)
  (if (not (find-package "STEPPER_MSG"))
    (make-package "STEPPER_MSG"))
  (shadow 'Stepper_Status (find-package "STEPPER_MSG")))
(unless (find-package "STEPPER_MSG::STEPPER_STATUS")
  (make-package "STEPPER_MSG::STEPPER_STATUS"))

(in-package "ROS")
;;//! \htmlinclude Stepper_Status.msg.html


(defclass stepper_msg::Stepper_Status
  :super ros::object
  :slots (_position_steps _speed_steps_per_second _direction_forward ))

(defmethod stepper_msg::Stepper_Status
  (:init
   (&key
    ((:position_steps __position_steps) 0)
    ((:speed_steps_per_second __speed_steps_per_second) 0)
    ((:direction_forward __direction_forward) nil)
    )
   (send-super :init)
   (setq _position_steps (round __position_steps))
   (setq _speed_steps_per_second (round __speed_steps_per_second))
   (setq _direction_forward __direction_forward)
   self)
  (:position_steps
   (&optional __position_steps)
   (if __position_steps (setq _position_steps __position_steps)) _position_steps)
  (:speed_steps_per_second
   (&optional __speed_steps_per_second)
   (if __speed_steps_per_second (setq _speed_steps_per_second __speed_steps_per_second)) _speed_steps_per_second)
  (:direction_forward
   (&optional __direction_forward)
   (if __direction_forward (setq _direction_forward __direction_forward)) _direction_forward)
  (:serialization-length
   ()
   (+
    ;; uint32 _position_steps
    4
    ;; uint32 _speed_steps_per_second
    4
    ;; bool _direction_forward
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _position_steps
       (write-long _position_steps s)
     ;; uint32 _speed_steps_per_second
       (write-long _speed_steps_per_second s)
     ;; bool _direction_forward
       (if _direction_forward (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _position_steps
     (setq _position_steps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _speed_steps_per_second
     (setq _speed_steps_per_second (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _direction_forward
     (setq _direction_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get stepper_msg::Stepper_Status :md5sum-) "3a08b832803b195f0f005fead32aedac")
(setf (get stepper_msg::Stepper_Status :datatype-) "stepper_msg/Stepper_Status")
(setf (get stepper_msg::Stepper_Status :definition-)
      "uint32 position_steps
uint32 speed_steps_per_second
bool direction_forward
")



(provide :stepper_msg/Stepper_Status "3a08b832803b195f0f005fead32aedac")


