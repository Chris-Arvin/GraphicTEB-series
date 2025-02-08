
(cl:in-package :asdf)

(defsystem "pedsim_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :pedsim_msgs-msg
)
  :components ((:file "_package")
    (:file "GetAgentState" :depends-on ("_package_GetAgentState"))
    (:file "_package_GetAgentState" :depends-on ("_package"))
    (:file "GetAllAgentsState" :depends-on ("_package_GetAllAgentsState"))
    (:file "_package_GetAllAgentsState" :depends-on ("_package"))
    (:file "SetAgentState" :depends-on ("_package_SetAgentState"))
    (:file "_package_SetAgentState" :depends-on ("_package"))
    (:file "SetAllAgentsState" :depends-on ("_package_SetAllAgentsState"))
    (:file "_package_SetAllAgentsState" :depends-on ("_package"))
  ))