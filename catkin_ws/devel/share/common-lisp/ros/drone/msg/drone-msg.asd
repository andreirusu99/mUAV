
(cl:in-package :asdf)

(defsystem "drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Attitude" :depends-on ("_package_Attitude"))
    (:file "_package_Attitude" :depends-on ("_package"))
    (:file "ControlAxes" :depends-on ("_package_ControlAxes"))
    (:file "_package_ControlAxes" :depends-on ("_package"))
    (:file "Power" :depends-on ("_package_Power"))
    (:file "_package_Power" :depends-on ("_package"))
  ))