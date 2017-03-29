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
  :slots (_position_steps _speed_steps_per_second _direction_forward _enabled _errors ))

(defmethod stepper_msg::Stepper_Status
  (:init
   (&key
    ((:position_steps __position_steps) 0)
    ((:speed_steps_per_second __speed_steps_per_second) 0)
    ((:direction_forward __direction_forward) nil)
    ((:enabled __enabled) nil)
    ((:errors __errors) "")
    )
   (send-super :init)
   (setq _position_steps (round __position_steps))
   (setq _speed_steps_per_second (round __speed_steps_per_second))
   (setq _direction_forward __direction_forward)
   (setq _enabled __enabled)
   (setq _errors (string __errors))
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
  (:enabled
   (&optional __enabled)
   (if __enabled (setq _enabled __enabled)) _enabled)
  (:errors
   (&optional __errors)
   (if __errors (setq _errors __errors)) _errors)
  (:serialization-length
   ()
   (+
    ;; int64 _position_steps
    8
    ;; int32 _speed_steps_per_second
    4
    ;; bool _direction_forward
    1
    ;; bool _enabled
    1
    ;; string _errors
    4 (length _errors)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _position_steps
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _position_steps (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _position_steps) (= (length (_position_steps . bv)) 2)) ;; bignum
              (write-long (ash (elt (_position_steps . bv) 0) 0) s)
              (write-long (ash (elt (_position_steps . bv) 1) -1) s))
             ((and (class _position_steps) (= (length (_position_steps . bv)) 1)) ;; big1
              (write-long (elt (_position_steps . bv) 0) s)
              (write-long (if (>= _position_steps 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _position_steps s)(write-long (if (>= _position_steps 0) 0 #xffffffff) s)))
     ;; int32 _speed_steps_per_second
       (write-long _speed_steps_per_second s)
     ;; bool _direction_forward
       (if _direction_forward (write-byte -1 s) (write-byte 0 s))
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;; string _errors
       (write-long (length _errors) s) (princ _errors s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _position_steps
#+(or :alpha :irix6 :x86_64)
      (setf _position_steps (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _position_steps (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int32 _speed_steps_per_second
     (setq _speed_steps_per_second (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _direction_forward
     (setq _direction_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _errors
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _errors (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get stepper_msg::Stepper_Status :md5sum-) "8b977b7753d25b4757850d088d3939b1")
(setf (get stepper_msg::Stepper_Status :datatype-) "stepper_msg/Stepper_Status")
(setf (get stepper_msg::Stepper_Status :definition-)
      "int64 position_steps
int32 speed_steps_per_second
bool direction_forward
bool enabled
string errors
")



(provide :stepper_msg/Stepper_Status "8b977b7753d25b4757850d088d3939b1")


