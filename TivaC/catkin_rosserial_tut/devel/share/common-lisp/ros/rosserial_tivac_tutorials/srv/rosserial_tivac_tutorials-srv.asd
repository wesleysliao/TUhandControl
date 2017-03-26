
(cl:in-package :asdf)

(defsystem "rosserial_tivac_tutorials-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ColorRGBA" :depends-on ("_package_ColorRGBA"))
    (:file "_package_ColorRGBA" :depends-on ("_package"))
  ))