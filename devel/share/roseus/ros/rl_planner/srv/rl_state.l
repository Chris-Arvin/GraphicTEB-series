;; Auto-generated. Do not edit!


(when (boundp 'rl_planner::rl_state)
  (if (not (find-package "RL_PLANNER"))
    (make-package "RL_PLANNER"))
  (shadow 'rl_state (find-package "RL_PLANNER")))
(unless (find-package "RL_PLANNER::RL_STATE")
  (make-package "RL_PLANNER::RL_STATE"))
(unless (find-package "RL_PLANNER::RL_STATEREQUEST")
  (make-package "RL_PLANNER::RL_STATEREQUEST"))
(unless (find-package "RL_PLANNER::RL_STATERESPONSE")
  (make-package "RL_PLANNER::RL_STATERESPONSE"))

(in-package "ROS")

(if (not (find-package "VISUALIZATION_MSGS"))
  (ros::roseus-add-msgs "visualization_msgs"))




(defclass rl_planner::rl_stateRequest
  :super ros::object
  :slots (_map_static_position _map_dynamic_velocity_x _map_dynamic_velocity_y _trajectories _vx _vy _last_static_safety_margin _last_dynamic_safety_margin ))

(defmethod rl_planner::rl_stateRequest
  (:init
   (&key
    ((:map_static_position __map_static_position) (make-array 0 :initial-element 0 :element-type :integer))
    ((:map_dynamic_velocity_x __map_dynamic_velocity_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:map_dynamic_velocity_y __map_dynamic_velocity_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:trajectories __trajectories) (instance visualization_msgs::MarkerArray :init))
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:last_static_safety_margin __last_static_safety_margin) 0.0)
    ((:last_dynamic_safety_margin __last_dynamic_safety_margin) 0.0)
    )
   (send-super :init)
   (setq _map_static_position __map_static_position)
   (setq _map_dynamic_velocity_x __map_dynamic_velocity_x)
   (setq _map_dynamic_velocity_y __map_dynamic_velocity_y)
   (setq _trajectories __trajectories)
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _last_static_safety_margin (float __last_static_safety_margin))
   (setq _last_dynamic_safety_margin (float __last_dynamic_safety_margin))
   self)
  (:map_static_position
   (&optional __map_static_position)
   (if __map_static_position (setq _map_static_position __map_static_position)) _map_static_position)
  (:map_dynamic_velocity_x
   (&optional __map_dynamic_velocity_x)
   (if __map_dynamic_velocity_x (setq _map_dynamic_velocity_x __map_dynamic_velocity_x)) _map_dynamic_velocity_x)
  (:map_dynamic_velocity_y
   (&optional __map_dynamic_velocity_y)
   (if __map_dynamic_velocity_y (setq _map_dynamic_velocity_y __map_dynamic_velocity_y)) _map_dynamic_velocity_y)
  (:trajectories
   (&rest __trajectories)
   (if (keywordp (car __trajectories))
       (send* _trajectories __trajectories)
     (progn
       (if __trajectories (setq _trajectories (car __trajectories)))
       _trajectories)))
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:last_static_safety_margin
   (&optional __last_static_safety_margin)
   (if __last_static_safety_margin (setq _last_static_safety_margin __last_static_safety_margin)) _last_static_safety_margin)
  (:last_dynamic_safety_margin
   (&optional __last_dynamic_safety_margin)
   (if __last_dynamic_safety_margin (setq _last_dynamic_safety_margin __last_dynamic_safety_margin)) _last_dynamic_safety_margin)
  (:serialization-length
   ()
   (+
    ;; int32[] _map_static_position
    (* 4    (length _map_static_position)) 4
    ;; float32[] _map_dynamic_velocity_x
    (* 4    (length _map_dynamic_velocity_x)) 4
    ;; float32[] _map_dynamic_velocity_y
    (* 4    (length _map_dynamic_velocity_y)) 4
    ;; visualization_msgs/MarkerArray _trajectories
    (send _trajectories :serialization-length)
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; float32 _last_static_safety_margin
    4
    ;; float32 _last_dynamic_safety_margin
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _map_static_position
     (write-long (length _map_static_position) s)
     (dotimes (i (length _map_static_position))
       (write-long (elt _map_static_position i) s)
       )
     ;; float32[] _map_dynamic_velocity_x
     (write-long (length _map_dynamic_velocity_x) s)
     (dotimes (i (length _map_dynamic_velocity_x))
       (sys::poke (elt _map_dynamic_velocity_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _map_dynamic_velocity_y
     (write-long (length _map_dynamic_velocity_y) s)
     (dotimes (i (length _map_dynamic_velocity_y))
       (sys::poke (elt _map_dynamic_velocity_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; visualization_msgs/MarkerArray _trajectories
       (send _trajectories :serialize s)
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _last_static_safety_margin
       (sys::poke _last_static_safety_margin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _last_dynamic_safety_margin
       (sys::poke _last_dynamic_safety_margin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _map_static_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_static_position (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _map_static_position i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _map_dynamic_velocity_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_dynamic_velocity_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _map_dynamic_velocity_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _map_dynamic_velocity_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_dynamic_velocity_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _map_dynamic_velocity_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; visualization_msgs/MarkerArray _trajectories
     (send _trajectories :deserialize buf ptr-) (incf ptr- (send _trajectories :serialization-length))
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _last_static_safety_margin
     (setq _last_static_safety_margin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _last_dynamic_safety_margin
     (setq _last_dynamic_safety_margin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rl_planner::rl_stateResponse
  :super ros::object
  :slots (_static_safety_margin _dynamic_safety_margin ))

(defmethod rl_planner::rl_stateResponse
  (:init
   (&key
    ((:static_safety_margin __static_safety_margin) 0.0)
    ((:dynamic_safety_margin __dynamic_safety_margin) 0.0)
    )
   (send-super :init)
   (setq _static_safety_margin (float __static_safety_margin))
   (setq _dynamic_safety_margin (float __dynamic_safety_margin))
   self)
  (:static_safety_margin
   (&optional __static_safety_margin)
   (if __static_safety_margin (setq _static_safety_margin __static_safety_margin)) _static_safety_margin)
  (:dynamic_safety_margin
   (&optional __dynamic_safety_margin)
   (if __dynamic_safety_margin (setq _dynamic_safety_margin __dynamic_safety_margin)) _dynamic_safety_margin)
  (:serialization-length
   ()
   (+
    ;; float32 _static_safety_margin
    4
    ;; float32 _dynamic_safety_margin
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _static_safety_margin
       (sys::poke _static_safety_margin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dynamic_safety_margin
       (sys::poke _dynamic_safety_margin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _static_safety_margin
     (setq _static_safety_margin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dynamic_safety_margin
     (setq _dynamic_safety_margin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rl_planner::rl_state
  :super ros::object
  :slots ())

(setf (get rl_planner::rl_state :md5sum-) "e0541c399b3508e509e9e93d705eab77")
(setf (get rl_planner::rl_state :datatype-) "rl_planner/rl_state")
(setf (get rl_planner::rl_state :request) rl_planner::rl_stateRequest)
(setf (get rl_planner::rl_state :response) rl_planner::rl_stateResponse)

(defmethod rl_planner::rl_stateRequest
  (:response () (instance rl_planner::rl_stateResponse :init)))

(setf (get rl_planner::rl_stateRequest :md5sum-) "e0541c399b3508e509e9e93d705eab77")
(setf (get rl_planner::rl_stateRequest :datatype-) "rl_planner/rl_stateRequest")
(setf (get rl_planner::rl_stateRequest :definition-)
      "# Request 部分
int32[]  map_static_position
float32[]  map_dynamic_velocity_x
float32[]  map_dynamic_velocity_y
visualization_msgs/MarkerArray trajectories
float32 vx
float32 vy
float32 last_static_safety_margin
float32 last_dynamic_safety_margin

================================================================================
MSG: visualization_msgs/MarkerArray
Marker[] markers

================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials

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
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
---
# Response 部分
float32 static_safety_margin
float32 dynamic_safety_margin
")

(setf (get rl_planner::rl_stateResponse :md5sum-) "e0541c399b3508e509e9e93d705eab77")
(setf (get rl_planner::rl_stateResponse :datatype-) "rl_planner/rl_stateResponse")
(setf (get rl_planner::rl_stateResponse :definition-)
      "# Request 部分
int32[]  map_static_position
float32[]  map_dynamic_velocity_x
float32[]  map_dynamic_velocity_y
visualization_msgs/MarkerArray trajectories
float32 vx
float32 vy
float32 last_static_safety_margin
float32 last_dynamic_safety_margin

================================================================================
MSG: visualization_msgs/MarkerArray
Marker[] markers

================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials

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
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
---
# Response 部分
float32 static_safety_margin
float32 dynamic_safety_margin
")



(provide :rl_planner/rl_state "e0541c399b3508e509e9e93d705eab77")


