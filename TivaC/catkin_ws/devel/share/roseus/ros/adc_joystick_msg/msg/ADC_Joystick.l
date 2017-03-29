;; Auto-generated. Do not edit!


(when (boundp 'adc_joystick_msg::ADC_Joystick)
  (if (not (find-package "ADC_JOYSTICK_MSG"))
    (make-package "ADC_JOYSTICK_MSG"))
  (shadow 'ADC_Joystick (find-package "ADC_JOYSTICK_MSG")))
(unless (find-package "ADC_JOYSTICK_MSG::ADC_JOYSTICK")
  (make-package "ADC_JOYSTICK_MSG::ADC_JOYSTICK"))

(in-package "ROS")
;;//! \htmlinclude ADC_Joystick.msg.html


(defclass adc_joystick_msg::ADC_Joystick
  :super ros::object
  :slots (_select _x_axis_raw _x_axis_zero _y_axis_raw _y_axis_zero ))

(defmethod adc_joystick_msg::ADC_Joystick
  (:init
   (&key
    ((:select __select) nil)
    ((:x_axis_raw __x_axis_raw) 0)
    ((:x_axis_zero __x_axis_zero) 0)
    ((:y_axis_raw __y_axis_raw) 0)
    ((:y_axis_zero __y_axis_zero) 0)
    )
   (send-super :init)
   (setq _select __select)
   (setq _x_axis_raw (round __x_axis_raw))
   (setq _x_axis_zero (round __x_axis_zero))
   (setq _y_axis_raw (round __y_axis_raw))
   (setq _y_axis_zero (round __y_axis_zero))
   self)
  (:select
   (&optional __select)
   (if __select (setq _select __select)) _select)
  (:x_axis_raw
   (&optional __x_axis_raw)
   (if __x_axis_raw (setq _x_axis_raw __x_axis_raw)) _x_axis_raw)
  (:x_axis_zero
   (&optional __x_axis_zero)
   (if __x_axis_zero (setq _x_axis_zero __x_axis_zero)) _x_axis_zero)
  (:y_axis_raw
   (&optional __y_axis_raw)
   (if __y_axis_raw (setq _y_axis_raw __y_axis_raw)) _y_axis_raw)
  (:y_axis_zero
   (&optional __y_axis_zero)
   (if __y_axis_zero (setq _y_axis_zero __y_axis_zero)) _y_axis_zero)
  (:serialization-length
   ()
   (+
    ;; bool _select
    1
    ;; int16 _x_axis_raw
    2
    ;; int16 _x_axis_zero
    2
    ;; int16 _y_axis_raw
    2
    ;; int16 _y_axis_zero
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _select
       (if _select (write-byte -1 s) (write-byte 0 s))
     ;; int16 _x_axis_raw
       (write-word _x_axis_raw s)
     ;; int16 _x_axis_zero
       (write-word _x_axis_zero s)
     ;; int16 _y_axis_raw
       (write-word _y_axis_raw s)
     ;; int16 _y_axis_zero
       (write-word _y_axis_zero s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _select
     (setq _select (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _x_axis_raw
     (setq _x_axis_raw (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _x_axis_zero
     (setq _x_axis_zero (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y_axis_raw
     (setq _y_axis_raw (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y_axis_zero
     (setq _y_axis_zero (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get adc_joystick_msg::ADC_Joystick :md5sum-) "10957cbdb8481676f0a00f626dbd9899")
(setf (get adc_joystick_msg::ADC_Joystick :datatype-) "adc_joystick_msg/ADC_Joystick")
(setf (get adc_joystick_msg::ADC_Joystick :definition-)
      "bool select
int16 x_axis_raw
int16 x_axis_zero
int16 y_axis_raw
int16 y_axis_zero


")



(provide :adc_joystick_msg/ADC_Joystick "10957cbdb8481676f0a00f626dbd9899")


