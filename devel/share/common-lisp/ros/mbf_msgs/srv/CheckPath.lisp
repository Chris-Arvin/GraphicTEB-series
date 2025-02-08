; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-srv)


;//! \htmlinclude CheckPath-request.msg.html

(cl:defclass <CheckPath-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path))
   (safety_dist
    :reader safety_dist
    :initarg :safety_dist
    :type cl:float
    :initform 0.0)
   (lethal_cost_mult
    :reader lethal_cost_mult
    :initarg :lethal_cost_mult
    :type cl:float
    :initform 0.0)
   (inscrib_cost_mult
    :reader inscrib_cost_mult
    :initarg :inscrib_cost_mult
    :type cl:float
    :initform 0.0)
   (unknown_cost_mult
    :reader unknown_cost_mult
    :initarg :unknown_cost_mult
    :type cl:float
    :initform 0.0)
   (costmap
    :reader costmap
    :initarg :costmap
    :type cl:fixnum
    :initform 0)
   (return_on
    :reader return_on
    :initarg :return_on
    :type cl:fixnum
    :initform 0)
   (skip_poses
    :reader skip_poses
    :initarg :skip_poses
    :type cl:fixnum
    :initform 0)
   (path_cells_only
    :reader path_cells_only
    :initarg :path_cells_only
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckPath-request (<CheckPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-srv:<CheckPath-request> is deprecated: use mbf_msgs-srv:CheckPath-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:path-val is deprecated.  Use mbf_msgs-srv:path instead.")
  (path m))

(cl:ensure-generic-function 'safety_dist-val :lambda-list '(m))
(cl:defmethod safety_dist-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:safety_dist-val is deprecated.  Use mbf_msgs-srv:safety_dist instead.")
  (safety_dist m))

(cl:ensure-generic-function 'lethal_cost_mult-val :lambda-list '(m))
(cl:defmethod lethal_cost_mult-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:lethal_cost_mult-val is deprecated.  Use mbf_msgs-srv:lethal_cost_mult instead.")
  (lethal_cost_mult m))

(cl:ensure-generic-function 'inscrib_cost_mult-val :lambda-list '(m))
(cl:defmethod inscrib_cost_mult-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:inscrib_cost_mult-val is deprecated.  Use mbf_msgs-srv:inscrib_cost_mult instead.")
  (inscrib_cost_mult m))

(cl:ensure-generic-function 'unknown_cost_mult-val :lambda-list '(m))
(cl:defmethod unknown_cost_mult-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:unknown_cost_mult-val is deprecated.  Use mbf_msgs-srv:unknown_cost_mult instead.")
  (unknown_cost_mult m))

(cl:ensure-generic-function 'costmap-val :lambda-list '(m))
(cl:defmethod costmap-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:costmap-val is deprecated.  Use mbf_msgs-srv:costmap instead.")
  (costmap m))

(cl:ensure-generic-function 'return_on-val :lambda-list '(m))
(cl:defmethod return_on-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:return_on-val is deprecated.  Use mbf_msgs-srv:return_on instead.")
  (return_on m))

(cl:ensure-generic-function 'skip_poses-val :lambda-list '(m))
(cl:defmethod skip_poses-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:skip_poses-val is deprecated.  Use mbf_msgs-srv:skip_poses instead.")
  (skip_poses m))

(cl:ensure-generic-function 'path_cells_only-val :lambda-list '(m))
(cl:defmethod path_cells_only-val ((m <CheckPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:path_cells_only-val is deprecated.  Use mbf_msgs-srv:path_cells_only instead.")
  (path_cells_only m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CheckPath-request>)))
    "Constants for message type '<CheckPath-request>"
  '((:LOCAL_COSTMAP . 1)
    (:GLOBAL_COSTMAP . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CheckPath-request)))
    "Constants for message type 'CheckPath-request"
  '((:LOCAL_COSTMAP . 1)
    (:GLOBAL_COSTMAP . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPath-request>) ostream)
  "Serializes a message object of type '<CheckPath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'safety_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lethal_cost_mult))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'inscrib_cost_mult))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'unknown_cost_mult))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'costmap)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'return_on)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'skip_poses)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'path_cells_only) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPath-request>) istream)
  "Deserializes a message object of type '<CheckPath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'safety_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lethal_cost_mult) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inscrib_cost_mult) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'unknown_cost_mult) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'costmap)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'return_on)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'skip_poses)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'path_cells_only) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPath-request>)))
  "Returns string type for a service object of type '<CheckPath-request>"
  "mbf_msgs/CheckPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPath-request)))
  "Returns string type for a service object of type 'CheckPath-request"
  "mbf_msgs/CheckPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPath-request>)))
  "Returns md5sum for a message object of type '<CheckPath-request>"
  "59ef27f5ce7e3cc3c7f7a0f823bca55c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPath-request)))
  "Returns md5sum for a message object of type 'CheckPath-request"
  "59ef27f5ce7e3cc3c7f7a0f823bca55c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPath-request>)))
  "Returns full string definition for message of type '<CheckPath-request>"
  (cl:format cl:nil "uint8                      LOCAL_COSTMAP  = 1~%uint8                      GLOBAL_COSTMAP = 2~%~%nav_msgs/Path              path              # the path to be checked after transforming to costmap frame~%float32                    safety_dist       # minimum distance allowed to the closest obstacle (footprint padding)~%float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)~%float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)~%float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)~%uint8                      costmap           # costmap in which to check the pose~%uint8                      return_on         # abort check on finding a pose with this state or worse (zero is ignored)~%uint8                      skip_poses        # skip this number of poses between checks, to speedup processing~%bool                       path_cells_only   # check only cells directly traversed by the path, ignoring robot footprint~%                                             # (if true, safety_dist is ignored)~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPath-request)))
  "Returns full string definition for message of type 'CheckPath-request"
  (cl:format cl:nil "uint8                      LOCAL_COSTMAP  = 1~%uint8                      GLOBAL_COSTMAP = 2~%~%nav_msgs/Path              path              # the path to be checked after transforming to costmap frame~%float32                    safety_dist       # minimum distance allowed to the closest obstacle (footprint padding)~%float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)~%float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)~%float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)~%uint8                      costmap           # costmap in which to check the pose~%uint8                      return_on         # abort check on finding a pose with this state or worse (zero is ignored)~%uint8                      skip_poses        # skip this number of poses between checks, to speedup processing~%bool                       path_cells_only   # check only cells directly traversed by the path, ignoring robot footprint~%                                             # (if true, safety_dist is ignored)~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     4
     4
     4
     4
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPath-request
    (cl:cons ':path (path msg))
    (cl:cons ':safety_dist (safety_dist msg))
    (cl:cons ':lethal_cost_mult (lethal_cost_mult msg))
    (cl:cons ':inscrib_cost_mult (inscrib_cost_mult msg))
    (cl:cons ':unknown_cost_mult (unknown_cost_mult msg))
    (cl:cons ':costmap (costmap msg))
    (cl:cons ':return_on (return_on msg))
    (cl:cons ':skip_poses (skip_poses msg))
    (cl:cons ':path_cells_only (path_cells_only msg))
))
;//! \htmlinclude CheckPath-response.msg.html

(cl:defclass <CheckPath-response> (roslisp-msg-protocol:ros-message)
  ((last_checked
    :reader last_checked
    :initarg :last_checked
    :type cl:integer
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (cost
    :reader cost
    :initarg :cost
    :type cl:integer
    :initform 0))
)

(cl:defclass CheckPath-response (<CheckPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-srv:<CheckPath-response> is deprecated: use mbf_msgs-srv:CheckPath-response instead.")))

(cl:ensure-generic-function 'last_checked-val :lambda-list '(m))
(cl:defmethod last_checked-val ((m <CheckPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:last_checked-val is deprecated.  Use mbf_msgs-srv:last_checked instead.")
  (last_checked m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <CheckPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:state-val is deprecated.  Use mbf_msgs-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'cost-val :lambda-list '(m))
(cl:defmethod cost-val ((m <CheckPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:cost-val is deprecated.  Use mbf_msgs-srv:cost instead.")
  (cost m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CheckPath-response>)))
    "Constants for message type '<CheckPath-response>"
  '((:FREE . 0)
    (:INSCRIBED . 1)
    (:LETHAL . 2)
    (:UNKNOWN . 3)
    (:OUTSIDE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CheckPath-response)))
    "Constants for message type 'CheckPath-response"
  '((:FREE . 0)
    (:INSCRIBED . 1)
    (:LETHAL . 2)
    (:UNKNOWN . 3)
    (:OUTSIDE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPath-response>) ostream)
  "Serializes a message object of type '<CheckPath-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'last_checked)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'last_checked)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'last_checked)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'last_checked)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cost)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPath-response>) istream)
  "Deserializes a message object of type '<CheckPath-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'last_checked)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'last_checked)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'last_checked)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'last_checked)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cost)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPath-response>)))
  "Returns string type for a service object of type '<CheckPath-response>"
  "mbf_msgs/CheckPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPath-response)))
  "Returns string type for a service object of type 'CheckPath-response"
  "mbf_msgs/CheckPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPath-response>)))
  "Returns md5sum for a message object of type '<CheckPath-response>"
  "59ef27f5ce7e3cc3c7f7a0f823bca55c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPath-response)))
  "Returns md5sum for a message object of type 'CheckPath-response"
  "59ef27f5ce7e3cc3c7f7a0f823bca55c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPath-response>)))
  "Returns full string definition for message of type '<CheckPath-response>"
  (cl:format cl:nil "uint8                      FREE      =  0    # path is completely in traversable space~%uint8                      INSCRIBED =  1    # path is partially in inscribed space~%uint8                      LETHAL    =  2    # path is partially in collision~%uint8                      UNKNOWN   =  3    # path is partially in unknown space~%uint8                      OUTSIDE   =  4    # path is partially outside the map~%~%uint32                     last_checked      # index of the first pose along the path with return_on state or worse~%uint8                      state             # path worst state (until last_checked): FREE, INFLATED, LETHAL, UNKNOWN...~%uint32                     cost              # cost of all cells traversed by path within footprint padded by safety_dist~%                                             # or just by the path if path_cells_only is true~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPath-response)))
  "Returns full string definition for message of type 'CheckPath-response"
  (cl:format cl:nil "uint8                      FREE      =  0    # path is completely in traversable space~%uint8                      INSCRIBED =  1    # path is partially in inscribed space~%uint8                      LETHAL    =  2    # path is partially in collision~%uint8                      UNKNOWN   =  3    # path is partially in unknown space~%uint8                      OUTSIDE   =  4    # path is partially outside the map~%~%uint32                     last_checked      # index of the first pose along the path with return_on state or worse~%uint8                      state             # path worst state (until last_checked): FREE, INFLATED, LETHAL, UNKNOWN...~%uint32                     cost              # cost of all cells traversed by path within footprint padded by safety_dist~%                                             # or just by the path if path_cells_only is true~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPath-response>))
  (cl:+ 0
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPath-response
    (cl:cons ':last_checked (last_checked msg))
    (cl:cons ':state (state msg))
    (cl:cons ':cost (cost msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckPath)))
  'CheckPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckPath)))
  'CheckPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPath)))
  "Returns string type for a service object of type '<CheckPath>"
  "mbf_msgs/CheckPath")