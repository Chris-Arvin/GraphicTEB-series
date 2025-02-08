; Auto-generated. Do not edit!


(cl:in-package pedsim_srvs-srv)


;//! \htmlinclude SetAllAgentsState-request.msg.html

(cl:defclass <SetAllAgentsState-request> (roslisp-msg-protocol:ros-message)
  ((agent_states
    :reader agent_states
    :initarg :agent_states
    :type pedsim_msgs-msg:AgentStates
    :initform (cl:make-instance 'pedsim_msgs-msg:AgentStates)))
)

(cl:defclass SetAllAgentsState-request (<SetAllAgentsState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAllAgentsState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAllAgentsState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_srvs-srv:<SetAllAgentsState-request> is deprecated: use pedsim_srvs-srv:SetAllAgentsState-request instead.")))

(cl:ensure-generic-function 'agent_states-val :lambda-list '(m))
(cl:defmethod agent_states-val ((m <SetAllAgentsState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_srvs-srv:agent_states-val is deprecated.  Use pedsim_srvs-srv:agent_states instead.")
  (agent_states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAllAgentsState-request>) ostream)
  "Serializes a message object of type '<SetAllAgentsState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'agent_states) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAllAgentsState-request>) istream)
  "Deserializes a message object of type '<SetAllAgentsState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'agent_states) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAllAgentsState-request>)))
  "Returns string type for a service object of type '<SetAllAgentsState-request>"
  "pedsim_srvs/SetAllAgentsStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAllAgentsState-request)))
  "Returns string type for a service object of type 'SetAllAgentsState-request"
  "pedsim_srvs/SetAllAgentsStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAllAgentsState-request>)))
  "Returns md5sum for a message object of type '<SetAllAgentsState-request>"
  "326e85f0f7b62adec56b45ab8b56826c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAllAgentsState-request)))
  "Returns md5sum for a message object of type 'SetAllAgentsState-request"
  "326e85f0f7b62adec56b45ab8b56826c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAllAgentsState-request>)))
  "Returns full string definition for message of type '<SetAllAgentsState-request>"
  (cl:format cl:nil "pedsim_msgs/AgentStates agent_states~%~%================================================================================~%MSG: pedsim_msgs/AgentStates~%Header header~%pedsim_msgs/AgentState[] agent_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/AgentState~%Header header~%uint64 id~%uint16 type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%~%# Use sensors package to control observability~%~%# Social State string constants~%string      TYPE_STANDING = \"standing\"~%string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"~%string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"~%string      TYPE_GROUP_MOVING = \"group_moving\"~%~%~%# Agent types~%# 0, 1 -> ordinary agents~%# 2 -> Robot~%# 3 -> standing/elderly agents~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAllAgentsState-request)))
  "Returns full string definition for message of type 'SetAllAgentsState-request"
  (cl:format cl:nil "pedsim_msgs/AgentStates agent_states~%~%================================================================================~%MSG: pedsim_msgs/AgentStates~%Header header~%pedsim_msgs/AgentState[] agent_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/AgentState~%Header header~%uint64 id~%uint16 type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%~%# Use sensors package to control observability~%~%# Social State string constants~%string      TYPE_STANDING = \"standing\"~%string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"~%string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"~%string      TYPE_GROUP_MOVING = \"group_moving\"~%~%~%# Agent types~%# 0, 1 -> ordinary agents~%# 2 -> Robot~%# 3 -> standing/elderly agents~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAllAgentsState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'agent_states))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAllAgentsState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAllAgentsState-request
    (cl:cons ':agent_states (agent_states msg))
))
;//! \htmlinclude SetAllAgentsState-response.msg.html

(cl:defclass <SetAllAgentsState-response> (roslisp-msg-protocol:ros-message)
  ((finished
    :reader finished
    :initarg :finished
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetAllAgentsState-response (<SetAllAgentsState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAllAgentsState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAllAgentsState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_srvs-srv:<SetAllAgentsState-response> is deprecated: use pedsim_srvs-srv:SetAllAgentsState-response instead.")))

(cl:ensure-generic-function 'finished-val :lambda-list '(m))
(cl:defmethod finished-val ((m <SetAllAgentsState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_srvs-srv:finished-val is deprecated.  Use pedsim_srvs-srv:finished instead.")
  (finished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAllAgentsState-response>) ostream)
  "Serializes a message object of type '<SetAllAgentsState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finished) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAllAgentsState-response>) istream)
  "Deserializes a message object of type '<SetAllAgentsState-response>"
    (cl:setf (cl:slot-value msg 'finished) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAllAgentsState-response>)))
  "Returns string type for a service object of type '<SetAllAgentsState-response>"
  "pedsim_srvs/SetAllAgentsStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAllAgentsState-response)))
  "Returns string type for a service object of type 'SetAllAgentsState-response"
  "pedsim_srvs/SetAllAgentsStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAllAgentsState-response>)))
  "Returns md5sum for a message object of type '<SetAllAgentsState-response>"
  "326e85f0f7b62adec56b45ab8b56826c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAllAgentsState-response)))
  "Returns md5sum for a message object of type 'SetAllAgentsState-response"
  "326e85f0f7b62adec56b45ab8b56826c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAllAgentsState-response>)))
  "Returns full string definition for message of type '<SetAllAgentsState-response>"
  (cl:format cl:nil "bool finished~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAllAgentsState-response)))
  "Returns full string definition for message of type 'SetAllAgentsState-response"
  (cl:format cl:nil "bool finished~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAllAgentsState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAllAgentsState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAllAgentsState-response
    (cl:cons ':finished (finished msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAllAgentsState)))
  'SetAllAgentsState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAllAgentsState)))
  'SetAllAgentsState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAllAgentsState)))
  "Returns string type for a service object of type '<SetAllAgentsState>"
  "pedsim_srvs/SetAllAgentsState")