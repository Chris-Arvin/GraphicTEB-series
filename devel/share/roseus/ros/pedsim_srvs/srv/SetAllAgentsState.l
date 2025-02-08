;; Auto-generated. Do not edit!


(when (boundp 'pedsim_srvs::SetAllAgentsState)
  (if (not (find-package "PEDSIM_SRVS"))
    (make-package "PEDSIM_SRVS"))
  (shadow 'SetAllAgentsState (find-package "PEDSIM_SRVS")))
(unless (find-package "PEDSIM_SRVS::SETALLAGENTSSTATE")
  (make-package "PEDSIM_SRVS::SETALLAGENTSSTATE"))
(unless (find-package "PEDSIM_SRVS::SETALLAGENTSSTATEREQUEST")
  (make-package "PEDSIM_SRVS::SETALLAGENTSSTATEREQUEST"))
(unless (find-package "PEDSIM_SRVS::SETALLAGENTSSTATERESPONSE")
  (make-package "PEDSIM_SRVS::SETALLAGENTSSTATERESPONSE"))

(in-package "ROS")

(if (not (find-package "PEDSIM_MSGS"))
  (ros::roseus-add-msgs "pedsim_msgs"))




(defclass pedsim_srvs::SetAllAgentsStateRequest
  :super ros::object
  :slots (_agent_states ))

(defmethod pedsim_srvs::SetAllAgentsStateRequest
  (:init
   (&key
    ((:agent_states __agent_states) (instance pedsim_msgs::AgentStates :init))
    )
   (send-super :init)
   (setq _agent_states __agent_states)
   self)
  (:agent_states
   (&rest __agent_states)
   (if (keywordp (car __agent_states))
       (send* _agent_states __agent_states)
     (progn
       (if __agent_states (setq _agent_states (car __agent_states)))
       _agent_states)))
  (:serialization-length
   ()
   (+
    ;; pedsim_msgs/AgentStates _agent_states
    (send _agent_states :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pedsim_msgs/AgentStates _agent_states
       (send _agent_states :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pedsim_msgs/AgentStates _agent_states
     (send _agent_states :deserialize buf ptr-) (incf ptr- (send _agent_states :serialization-length))
   ;;
   self)
  )

(defclass pedsim_srvs::SetAllAgentsStateResponse
  :super ros::object
  :slots (_finished ))

(defmethod pedsim_srvs::SetAllAgentsStateResponse
  (:init
   (&key
    ((:finished __finished) nil)
    )
   (send-super :init)
   (setq _finished __finished)
   self)
  (:finished
   (&optional (__finished :null))
   (if (not (eq __finished :null)) (setq _finished __finished)) _finished)
  (:serialization-length
   ()
   (+
    ;; bool _finished
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _finished
       (if _finished (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _finished
     (setq _finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass pedsim_srvs::SetAllAgentsState
  :super ros::object
  :slots ())

(setf (get pedsim_srvs::SetAllAgentsState :md5sum-) "326e85f0f7b62adec56b45ab8b56826c")
(setf (get pedsim_srvs::SetAllAgentsState :datatype-) "pedsim_srvs/SetAllAgentsState")
(setf (get pedsim_srvs::SetAllAgentsState :request) pedsim_srvs::SetAllAgentsStateRequest)
(setf (get pedsim_srvs::SetAllAgentsState :response) pedsim_srvs::SetAllAgentsStateResponse)

(defmethod pedsim_srvs::SetAllAgentsStateRequest
  (:response () (instance pedsim_srvs::SetAllAgentsStateResponse :init)))

(setf (get pedsim_srvs::SetAllAgentsStateRequest :md5sum-) "326e85f0f7b62adec56b45ab8b56826c")
(setf (get pedsim_srvs::SetAllAgentsStateRequest :datatype-) "pedsim_srvs/SetAllAgentsStateRequest")
(setf (get pedsim_srvs::SetAllAgentsStateRequest :definition-)
      "pedsim_msgs/AgentStates agent_states

================================================================================
MSG: pedsim_msgs/AgentStates
Header header
pedsim_msgs/AgentState[] agent_states

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
---
bool finished

")

(setf (get pedsim_srvs::SetAllAgentsStateResponse :md5sum-) "326e85f0f7b62adec56b45ab8b56826c")
(setf (get pedsim_srvs::SetAllAgentsStateResponse :datatype-) "pedsim_srvs/SetAllAgentsStateResponse")
(setf (get pedsim_srvs::SetAllAgentsStateResponse :definition-)
      "pedsim_msgs/AgentStates agent_states

================================================================================
MSG: pedsim_msgs/AgentStates
Header header
pedsim_msgs/AgentState[] agent_states

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
---
bool finished

")



(provide :pedsim_srvs/SetAllAgentsState "326e85f0f7b62adec56b45ab8b56826c")


