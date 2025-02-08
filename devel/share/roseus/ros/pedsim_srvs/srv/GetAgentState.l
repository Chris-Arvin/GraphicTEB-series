;; Auto-generated. Do not edit!


(when (boundp 'pedsim_srvs::GetAgentState)
  (if (not (find-package "PEDSIM_SRVS"))
    (make-package "PEDSIM_SRVS"))
  (shadow 'GetAgentState (find-package "PEDSIM_SRVS")))
(unless (find-package "PEDSIM_SRVS::GETAGENTSTATE")
  (make-package "PEDSIM_SRVS::GETAGENTSTATE"))
(unless (find-package "PEDSIM_SRVS::GETAGENTSTATEREQUEST")
  (make-package "PEDSIM_SRVS::GETAGENTSTATEREQUEST"))
(unless (find-package "PEDSIM_SRVS::GETAGENTSTATERESPONSE")
  (make-package "PEDSIM_SRVS::GETAGENTSTATERESPONSE"))

(in-package "ROS")



(if (not (find-package "PEDSIM_MSGS"))
  (ros::roseus-add-msgs "pedsim_msgs"))


(defclass pedsim_srvs::GetAgentStateRequest
  :super ros::object
  :slots (_agent_id ))

(defmethod pedsim_srvs::GetAgentStateRequest
  (:init
   (&key
    ((:agent_id __agent_id) 0)
    )
   (send-super :init)
   (setq _agent_id (round __agent_id))
   self)
  (:agent_id
   (&optional __agent_id)
   (if __agent_id (setq _agent_id __agent_id)) _agent_id)
  (:serialization-length
   ()
   (+
    ;; int16 _agent_id
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _agent_id
       (write-word _agent_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _agent_id
     (setq _agent_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass pedsim_srvs::GetAgentStateResponse
  :super ros::object
  :slots (_state ))

(defmethod pedsim_srvs::GetAgentStateResponse
  (:init
   (&key
    ((:state __state) (instance pedsim_msgs::AgentState :init))
    )
   (send-super :init)
   (setq _state __state)
   self)
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:serialization-length
   ()
   (+
    ;; pedsim_msgs/AgentState _state
    (send _state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pedsim_msgs/AgentState _state
       (send _state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pedsim_msgs/AgentState _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;;
   self)
  )

(defclass pedsim_srvs::GetAgentState
  :super ros::object
  :slots ())

(setf (get pedsim_srvs::GetAgentState :md5sum-) "506aed4cf0fa361a55600b1ac6b1f978")
(setf (get pedsim_srvs::GetAgentState :datatype-) "pedsim_srvs/GetAgentState")
(setf (get pedsim_srvs::GetAgentState :request) pedsim_srvs::GetAgentStateRequest)
(setf (get pedsim_srvs::GetAgentState :response) pedsim_srvs::GetAgentStateResponse)

(defmethod pedsim_srvs::GetAgentStateRequest
  (:response () (instance pedsim_srvs::GetAgentStateResponse :init)))

(setf (get pedsim_srvs::GetAgentStateRequest :md5sum-) "506aed4cf0fa361a55600b1ac6b1f978")
(setf (get pedsim_srvs::GetAgentStateRequest :datatype-) "pedsim_srvs/GetAgentStateRequest")
(setf (get pedsim_srvs::GetAgentStateRequest :definition-)
      "int16 agent_id
---
pedsim_msgs/AgentState state

================================================================================
MSG: pedsim_msgs/AgentState
Header header
uint64 id
uint16 type
string social_state
geometry_msgs/Pose pose
geometry_msgs/Twist twist
pedsim_msgs/AgentForce forces

# Use sensors package to control observability

# Social State string constants
string      TYPE_STANDING = \"standing\"
string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"
string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"
string      TYPE_GROUP_MOVING = \"group_moving\"


# Agent types
# 0, 1 -> ordinary agents
# 2 -> Robot
# 3 -> standing/elderly agents

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: pedsim_msgs/AgentForce
# Forces acting on an agent.

# Basic SFM forces.
geometry_msgs/Vector3 desired_force
geometry_msgs/Vector3 obstacle_force
geometry_msgs/Vector3 social_force

# Additional Group Forces
geometry_msgs/Vector3 group_coherence_force
geometry_msgs/Vector3 group_gaze_force
geometry_msgs/Vector3 group_repulsion_force

# Extra stabilization/custom forces.
geometry_msgs/Vector3 random_force
")

(setf (get pedsim_srvs::GetAgentStateResponse :md5sum-) "506aed4cf0fa361a55600b1ac6b1f978")
(setf (get pedsim_srvs::GetAgentStateResponse :datatype-) "pedsim_srvs/GetAgentStateResponse")
(setf (get pedsim_srvs::GetAgentStateResponse :definition-)
      "int16 agent_id
---
pedsim_msgs/AgentState state

================================================================================
MSG: pedsim_msgs/AgentState
Header header
uint64 id
uint16 type
string social_state
geometry_msgs/Pose pose
geometry_msgs/Twist twist
pedsim_msgs/AgentForce forces

# Use sensors package to control observability

# Social State string constants
string      TYPE_STANDING = \"standing\"
string      TYPE_INDIVIDUAL_MOVING = \"individual_moving\"
string      TYPE_WAITING_IN_QUEUE = \"waiting_in_queue\"
string      TYPE_GROUP_MOVING = \"group_moving\"


# Agent types
# 0, 1 -> ordinary agents
# 2 -> Robot
# 3 -> standing/elderly agents

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: pedsim_msgs/AgentForce
# Forces acting on an agent.

# Basic SFM forces.
geometry_msgs/Vector3 desired_force
geometry_msgs/Vector3 obstacle_force
geometry_msgs/Vector3 social_force

# Additional Group Forces
geometry_msgs/Vector3 group_coherence_force
geometry_msgs/Vector3 group_gaze_force
geometry_msgs/Vector3 group_repulsion_force

# Extra stabilization/custom forces.
geometry_msgs/Vector3 random_force
")



(provide :pedsim_srvs/GetAgentState "506aed4cf0fa361a55600b1ac6b1f978")


