
(cl:in-package :asdf)

(defsystem "arm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "dynamixel_srv" :depends-on ("_package_dynamixel_srv"))
    (:file "_package_dynamixel_srv" :depends-on ("_package"))
    (:file "stepper_srv" :depends-on ("_package_stepper_srv"))
    (:file "_package_stepper_srv" :depends-on ("_package"))
  ))