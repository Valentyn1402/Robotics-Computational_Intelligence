
(cl:in-package :asdf)

(defsystem "motor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorPWM" :depends-on ("_package_MotorPWM"))
    (:file "_package_MotorPWM" :depends-on ("_package"))
    (:file "MotorRPM" :depends-on ("_package_MotorRPM"))
    (:file "_package_MotorRPM" :depends-on ("_package"))
  ))