
(cl:in-package :asdf)

(defsystem "motion_capture-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "nokovStamped" :depends-on ("_package_nokovStamped"))
    (:file "_package_nokovStamped" :depends-on ("_package"))
  ))