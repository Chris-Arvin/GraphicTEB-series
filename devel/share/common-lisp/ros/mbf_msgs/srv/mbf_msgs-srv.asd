
(cl:in-package :asdf)

(defsystem "mbf_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "CheckPath" :depends-on ("_package_CheckPath"))
    (:file "_package_CheckPath" :depends-on ("_package"))
    (:file "CheckPoint" :depends-on ("_package_CheckPoint"))
    (:file "_package_CheckPoint" :depends-on ("_package"))
    (:file "CheckPose" :depends-on ("_package_CheckPose"))
    (:file "_package_CheckPose" :depends-on ("_package"))
  ))