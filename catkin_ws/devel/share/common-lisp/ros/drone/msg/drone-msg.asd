
(cl:in-package :asdf)

(defsystem "drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Attitude" :depends-on ("_package_Attitude"))
    (:file "_package_Attitude" :depends-on ("_package"))
    (:file "ControlAxes" :depends-on ("_package_ControlAxes"))
    (:file "_package_ControlAxes" :depends-on ("_package"))
    (:file "Smth" :depends-on ("_package_Smth"))
    (:file "_package_Smth" :depends-on ("_package"))
    (:file "abc" :depends-on ("_package_abc"))
    (:file "_package_abc" :depends-on ("_package"))
  ))