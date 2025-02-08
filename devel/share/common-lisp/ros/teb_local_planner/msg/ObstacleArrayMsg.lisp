; Auto-generated. Do not edit!


(cl:in-package teb_local_planner-msg)


;//! \htmlinclude ObstacleArrayMsg.msg.html

(cl:defclass <ObstacleArrayMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector teb_local_planner-msg:ObstacleMsg)
   :initform (cl:make-array 0 :element-type 'teb_local_planner-msg:ObstacleMsg :initial-element (cl:make-instance 'teb_local_planner-msg:ObstacleMsg))))
)

(cl:defclass ObstacleArrayMsg (<ObstacleArrayMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleArrayMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleArrayMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teb_local_planner-msg:<ObstacleArrayMsg> is deprecated: use teb_local_planner-msg:ObstacleArrayMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleArrayMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teb_local_planner-msg:header-val is deprecated.  Use teb_local_planner-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <ObstacleArrayMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teb_local_planner-msg:obstacles-val is deprecated.  Use teb_local_planner-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleArrayMsg>) ostream)
  "Serializes a message object of type '<ObstacleArrayMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleArrayMsg>) istream)
  "Deserializes a message object of type '<ObstacleArrayMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'teb_local_planner-msg:ObstacleMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleArrayMsg>)))
  "Returns string type for a message object of type '<ObstacleArrayMsg>"
  "teb_local_planner/ObstacleArrayMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleArrayMsg)))
  "Returns string type for a message object of type 'ObstacleArrayMsg"
  "teb_local_planner/ObstacleArrayMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleArrayMsg>)))
  "Returns md5sum for a message object of type '<ObstacleArrayMsg>"
  "8a1bdcde72c65ca7d3ce8ebf52d43516")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleArrayMsg)))
  "Returns md5sum for a message object of type 'ObstacleArrayMsg"
  "8a1bdcde72c65ca7d3ce8ebf52d43516")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleArrayMsg>)))
  "Returns full string definition for message of type '<ObstacleArrayMsg>"
  (cl:format cl:nil "# Message that contains a list of polygon shaped obstacles.~%# Special types:~%# Polygon with 1 vertex: Point obstacle~%# Polygon with 2 vertices: Line obstacle~%# Polygon with more than 2 vertices: First and last points are assumed to be connected~%~%std_msgs/Header header~%~%teb_local_planner/ObstacleMsg[] obstacles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: teb_local_planner/ObstacleMsg~%# Special types:~%# Polygon with 1 vertex: Point obstacle (you might also specify a non-zero value for radius)~%# Polygon with 2 vertices: Line obstacle~%# Polygon with more than 2 vertices: First and last points are assumed to be connected~%~%std_msgs/Header header~%~%# Obstacle footprint (polygon descriptions)~%geometry_msgs/Polygon polygon~%~%# Specify the radius for circular/point obstacles~%float64 radius~%~%# Obstacle ID~%# Specify IDs in order to provide (temporal) relationships~%# between obstacles among multiple messages.~%int64 id~%~%# Individual orientation (centroid)~%geometry_msgs/Quaternion orientation~%~%# Individual velocities (centroid)~%geometry_msgs/TwistWithCovariance velocities~%~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleArrayMsg)))
  "Returns full string definition for message of type 'ObstacleArrayMsg"
  (cl:format cl:nil "# Message that contains a list of polygon shaped obstacles.~%# Special types:~%# Polygon with 1 vertex: Point obstacle~%# Polygon with 2 vertices: Line obstacle~%# Polygon with more than 2 vertices: First and last points are assumed to be connected~%~%std_msgs/Header header~%~%teb_local_planner/ObstacleMsg[] obstacles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: teb_local_planner/ObstacleMsg~%# Special types:~%# Polygon with 1 vertex: Point obstacle (you might also specify a non-zero value for radius)~%# Polygon with 2 vertices: Line obstacle~%# Polygon with more than 2 vertices: First and last points are assumed to be connected~%~%std_msgs/Header header~%~%# Obstacle footprint (polygon descriptions)~%geometry_msgs/Polygon polygon~%~%# Specify the radius for circular/point obstacles~%float64 radius~%~%# Obstacle ID~%# Specify IDs in order to provide (temporal) relationships~%# between obstacles among multiple messages.~%int64 id~%~%# Individual orientation (centroid)~%geometry_msgs/Quaternion orientation~%~%# Individual velocities (centroid)~%geometry_msgs/TwistWithCovariance velocities~%~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleArrayMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleArrayMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleArrayMsg
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
