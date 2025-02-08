; Auto-generated. Do not edit!


(cl:in-package rl_planner-srv)


;//! \htmlinclude rl_state-request.msg.html

(cl:defclass <rl_state-request> (roslisp-msg-protocol:ros-message)
  ((map_static_position
    :reader map_static_position
    :initarg :map_static_position
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (map_dynamic_velocity_x
    :reader map_dynamic_velocity_x
    :initarg :map_dynamic_velocity_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (map_dynamic_velocity_y
    :reader map_dynamic_velocity_y
    :initarg :map_dynamic_velocity_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (trajectories
    :reader trajectories
    :initarg :trajectories
    :type visualization_msgs-msg:MarkerArray
    :initform (cl:make-instance 'visualization_msgs-msg:MarkerArray))
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0)
   (last_static_safety_margin
    :reader last_static_safety_margin
    :initarg :last_static_safety_margin
    :type cl:float
    :initform 0.0)
   (last_dynamic_safety_margin
    :reader last_dynamic_safety_margin
    :initarg :last_dynamic_safety_margin
    :type cl:float
    :initform 0.0))
)

(cl:defclass rl_state-request (<rl_state-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rl_state-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rl_state-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_planner-srv:<rl_state-request> is deprecated: use rl_planner-srv:rl_state-request instead.")))

(cl:ensure-generic-function 'map_static_position-val :lambda-list '(m))
(cl:defmethod map_static_position-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:map_static_position-val is deprecated.  Use rl_planner-srv:map_static_position instead.")
  (map_static_position m))

(cl:ensure-generic-function 'map_dynamic_velocity_x-val :lambda-list '(m))
(cl:defmethod map_dynamic_velocity_x-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:map_dynamic_velocity_x-val is deprecated.  Use rl_planner-srv:map_dynamic_velocity_x instead.")
  (map_dynamic_velocity_x m))

(cl:ensure-generic-function 'map_dynamic_velocity_y-val :lambda-list '(m))
(cl:defmethod map_dynamic_velocity_y-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:map_dynamic_velocity_y-val is deprecated.  Use rl_planner-srv:map_dynamic_velocity_y instead.")
  (map_dynamic_velocity_y m))

(cl:ensure-generic-function 'trajectories-val :lambda-list '(m))
(cl:defmethod trajectories-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:trajectories-val is deprecated.  Use rl_planner-srv:trajectories instead.")
  (trajectories m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:vx-val is deprecated.  Use rl_planner-srv:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:vy-val is deprecated.  Use rl_planner-srv:vy instead.")
  (vy m))

(cl:ensure-generic-function 'last_static_safety_margin-val :lambda-list '(m))
(cl:defmethod last_static_safety_margin-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:last_static_safety_margin-val is deprecated.  Use rl_planner-srv:last_static_safety_margin instead.")
  (last_static_safety_margin m))

(cl:ensure-generic-function 'last_dynamic_safety_margin-val :lambda-list '(m))
(cl:defmethod last_dynamic_safety_margin-val ((m <rl_state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:last_dynamic_safety_margin-val is deprecated.  Use rl_planner-srv:last_dynamic_safety_margin instead.")
  (last_dynamic_safety_margin m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rl_state-request>) ostream)
  "Serializes a message object of type '<rl_state-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map_static_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'map_static_position))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map_dynamic_velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'map_dynamic_velocity_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map_dynamic_velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'map_dynamic_velocity_y))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectories) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'last_static_safety_margin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'last_dynamic_safety_margin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rl_state-request>) istream)
  "Deserializes a message object of type '<rl_state-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map_static_position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map_static_position)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map_dynamic_velocity_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map_dynamic_velocity_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map_dynamic_velocity_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map_dynamic_velocity_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectories) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'last_static_safety_margin) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'last_dynamic_safety_margin) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rl_state-request>)))
  "Returns string type for a service object of type '<rl_state-request>"
  "rl_planner/rl_stateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rl_state-request)))
  "Returns string type for a service object of type 'rl_state-request"
  "rl_planner/rl_stateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rl_state-request>)))
  "Returns md5sum for a message object of type '<rl_state-request>"
  "e0541c399b3508e509e9e93d705eab77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rl_state-request)))
  "Returns md5sum for a message object of type 'rl_state-request"
  "e0541c399b3508e509e9e93d705eab77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rl_state-request>)))
  "Returns full string definition for message of type '<rl_state-request>"
  (cl:format cl:nil "# Request 部分~%int32[]  map_static_position~%float32[]  map_dynamic_velocity_x~%float32[]  map_dynamic_velocity_y~%visualization_msgs/MarkerArray trajectories~%float32 vx~%float32 vy~%float32 last_static_safety_margin~%float32 last_dynamic_safety_margin~%~%================================================================================~%MSG: visualization_msgs/MarkerArray~%Marker[] markers~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%uint8 ARROW=0~%uint8 CUBE=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 LINE_STRIP=4~%uint8 LINE_LIST=5~%uint8 CUBE_LIST=6~%uint8 SPHERE_LIST=7~%uint8 POINTS=8~%uint8 TEXT_VIEW_FACING=9~%uint8 MESH_RESOURCE=10~%uint8 TRIANGLE_LIST=11~%~%uint8 ADD=0~%uint8 MODIFY=0~%uint8 DELETE=2~%uint8 DELETEALL=3~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rl_state-request)))
  "Returns full string definition for message of type 'rl_state-request"
  (cl:format cl:nil "# Request 部分~%int32[]  map_static_position~%float32[]  map_dynamic_velocity_x~%float32[]  map_dynamic_velocity_y~%visualization_msgs/MarkerArray trajectories~%float32 vx~%float32 vy~%float32 last_static_safety_margin~%float32 last_dynamic_safety_margin~%~%================================================================================~%MSG: visualization_msgs/MarkerArray~%Marker[] markers~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%uint8 ARROW=0~%uint8 CUBE=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 LINE_STRIP=4~%uint8 LINE_LIST=5~%uint8 CUBE_LIST=6~%uint8 SPHERE_LIST=7~%uint8 POINTS=8~%uint8 TEXT_VIEW_FACING=9~%uint8 MESH_RESOURCE=10~%uint8 TRIANGLE_LIST=11~%~%uint8 ADD=0~%uint8 MODIFY=0~%uint8 DELETE=2~%uint8 DELETEALL=3~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rl_state-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map_static_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map_dynamic_velocity_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map_dynamic_velocity_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectories))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rl_state-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rl_state-request
    (cl:cons ':map_static_position (map_static_position msg))
    (cl:cons ':map_dynamic_velocity_x (map_dynamic_velocity_x msg))
    (cl:cons ':map_dynamic_velocity_y (map_dynamic_velocity_y msg))
    (cl:cons ':trajectories (trajectories msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':last_static_safety_margin (last_static_safety_margin msg))
    (cl:cons ':last_dynamic_safety_margin (last_dynamic_safety_margin msg))
))
;//! \htmlinclude rl_state-response.msg.html

(cl:defclass <rl_state-response> (roslisp-msg-protocol:ros-message)
  ((static_safety_margin
    :reader static_safety_margin
    :initarg :static_safety_margin
    :type cl:float
    :initform 0.0)
   (dynamic_safety_margin
    :reader dynamic_safety_margin
    :initarg :dynamic_safety_margin
    :type cl:float
    :initform 0.0))
)

(cl:defclass rl_state-response (<rl_state-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rl_state-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rl_state-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_planner-srv:<rl_state-response> is deprecated: use rl_planner-srv:rl_state-response instead.")))

(cl:ensure-generic-function 'static_safety_margin-val :lambda-list '(m))
(cl:defmethod static_safety_margin-val ((m <rl_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:static_safety_margin-val is deprecated.  Use rl_planner-srv:static_safety_margin instead.")
  (static_safety_margin m))

(cl:ensure-generic-function 'dynamic_safety_margin-val :lambda-list '(m))
(cl:defmethod dynamic_safety_margin-val ((m <rl_state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_planner-srv:dynamic_safety_margin-val is deprecated.  Use rl_planner-srv:dynamic_safety_margin instead.")
  (dynamic_safety_margin m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rl_state-response>) ostream)
  "Serializes a message object of type '<rl_state-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'static_safety_margin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dynamic_safety_margin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rl_state-response>) istream)
  "Deserializes a message object of type '<rl_state-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'static_safety_margin) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dynamic_safety_margin) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rl_state-response>)))
  "Returns string type for a service object of type '<rl_state-response>"
  "rl_planner/rl_stateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rl_state-response)))
  "Returns string type for a service object of type 'rl_state-response"
  "rl_planner/rl_stateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rl_state-response>)))
  "Returns md5sum for a message object of type '<rl_state-response>"
  "e0541c399b3508e509e9e93d705eab77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rl_state-response)))
  "Returns md5sum for a message object of type 'rl_state-response"
  "e0541c399b3508e509e9e93d705eab77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rl_state-response>)))
  "Returns full string definition for message of type '<rl_state-response>"
  (cl:format cl:nil "# Response 部分~%float32 static_safety_margin~%float32 dynamic_safety_margin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rl_state-response)))
  "Returns full string definition for message of type 'rl_state-response"
  (cl:format cl:nil "# Response 部分~%float32 static_safety_margin~%float32 dynamic_safety_margin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rl_state-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rl_state-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rl_state-response
    (cl:cons ':static_safety_margin (static_safety_margin msg))
    (cl:cons ':dynamic_safety_margin (dynamic_safety_margin msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rl_state)))
  'rl_state-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rl_state)))
  'rl_state-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rl_state)))
  "Returns string type for a service object of type '<rl_state>"
  "rl_planner/rl_state")