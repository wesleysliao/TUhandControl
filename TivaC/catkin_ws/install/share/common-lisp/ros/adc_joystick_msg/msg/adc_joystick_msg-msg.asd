
(cl:in-package :asdf)

(defsystem "adc_joystick_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ADC_Joystick" :depends-on ("_package_ADC_Joystick"))
    (:file "_package_ADC_Joystick" :depends-on ("_package"))
  ))