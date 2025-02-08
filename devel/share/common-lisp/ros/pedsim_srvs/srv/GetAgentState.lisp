; Auto-generated. Do not edit!


(cl:in-package pedsim_srvs-srv)


;//! \htmlinclude GetAgentState-request.msg.html

(cl:defclass <GetAgentState-request> (roslisp-msg-protocol:ros-message)
  ((agent_id
    :reader agent_id
    :initarg :agent_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetAgentState-request (<GetAgentState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAgentState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAgentState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_srvs-srv:<GetAgentState-request> is deprecated: use pedsim_srvs-srv:GetAgentState-request instead.")))

(cl:ensure-generic-function 'agent_id-val :lambda-list '(m))
(cl:defmethod agent_id-val ((m <GetAgentState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_srvs-srv:agent_id-val is deprecated.  Use pedsim_srvs-srv:agent_id instead.")
  (agent_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAgentState-request>) ostream)
  "Serializes a message object of type '<GetAgentState-request>"
  (cl:let* ((signed (cl:slot-value msg 'agent_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAgentState-request>) istream)
  "Deserializes a message object of type '<GetAgentState-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'agent_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAgentState-request>)))
  "Returns string type for a service object of type '<GetAgentState-request>"
  "pedsim_srvs/GetAgentStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentState-request)))
  "Returns string type for a service object of type 'GetAgentState-request"
  "pedsim_srvs/GetAgentStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAgentState-request>)))
  "Returns md5sum for a message object of type '<GetAgentState-request>"
  "506aed4cf0fa361a55600b1ac6b1f978")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAgentState-request)))
  "Returns md5sum for a message object of type 'GetAgentState-request"
  "506aed4cf0fa361a55600b1ac6b1f978")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAgentState-request>)))
  "Returns full string definition for message of type '<GetAgentState-request>"
  (cl:format cl:nil "int16 agent_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAgentState-request)))
  "Returns full string definition for message of type 'GetAgentState-request"
  (cl:format cl:nil "int16 agent_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAgentState-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAgentState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAgentState-request
    (cl:cons ':agent_id (agent_id msg))
))
;//! \htmlinclude GetAgentState-response.msg.html

(cl:defclass <GetAgentState-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type pedsim_msgs-msg:AgentState
    :initform (cl:make-instance 'pedsim_msgs-msg:AgentState)))
)

(cl:defclass GetAgentState-response (<GetAgentState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAgentState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAgentState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_srvs-srv:<GetAgentState-response> is deprecated: use pedsim_srvs-srv:GetAgentState-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GetAgentState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_srvs-srv:state-val is deprecated.  Use pedsim_srvs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAgentState-response>) ostream)
  "Serializes a message object of type '<GetAgentState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAgentState-response>) istream)
  "Deserializes a message object of type '<GetAgentState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAgentState-response>)))
  "Returns string type for a service object of type '<GetAgentState-response>"
  "pedsim_srvs/GetAgentStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentState-response)))
  "Returns string type for a service object of type 'GetAgentState-response"
  "pedsim_srvs/GetAgentStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAgentState-response>)))
  "Returns md5sum for a message object of type '<GetAgentState-response>"
  "506aed4cf0fa361a55600b1ac6b1f978")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAgentState-response)))
  "Returns md5sum for a message object of type 'GetAgentState-response"
  "506aed4cf0fa361a55600b1ac6b1f978")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAgentState-response>)))
  "Returns full string definition for message of type '<GetAgentState-response>"
  (cl:format cl:nil "pedsim_msgs/AgentState state~%~%================================================================================~%MSG: pedsim_msgs/AgentState~%Header header~%uint64 id~%uint16 type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%~%# Use sensors package to control observability~%~%# Social State string constants~%string      TYPE_STANDING = \"standing\"~%string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"~%string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"~%string      TYPE_GROUP_MOVING = \"group_moving\"~%~%~%# Agent types~%# 0, 1 -> ordinary agents~%# 2 -> Robot~%# 3 -> standing/elderly agents~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAgentState-response)))
  "Returns full string definition for message of type 'GetAgentState-response"
  (cl:format cl:nil "pedsim_msgs/AgentState state~%~%================================================================================~%MSG: pedsim_msgs/AgentState~%Header header~%uint64 id~%uint16 type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%~%# Use sensors package to control observability~%~%# Social State string constants~%string      TYPE_STANDING = \"standing\"~%string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"~%string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"~%string      TYPE_GROUP_MOVING = \"group_moving\"~%~%~%# Agent types~%# 0, 1 -> ordinary agents~%# 2 -> Robot~%# 3 -> standing/elderly agents~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAgentState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAgentState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAgentState-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAgentState)))
  'GetAgentState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAgentState)))
  'GetAgentState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentState)))
  "Returns string type for a service object of type '<GetAgentState>"
  "pedsim_srvs/GetAgentState")