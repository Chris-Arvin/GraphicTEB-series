; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-srv)


;//! \htmlinclude CheckPose-request.msg.html

(cl:defclass <CheckPose-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
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
   (current_pose
    :reader current_pose
    :initarg :current_pose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckPose-request (<CheckPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-srv:<CheckPose-request> is deprecated: use mbf_msgs-srv:CheckPose-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <CheckPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:pose-val is deprecated.  Use mbf_msgs-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'safety_dist-val :lambda-list '(m))
(cl:defmethod safety_dist-val ((m <CheckPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:safety_dist-val is deprecated.  Use mbf_msgs-srv:safety_dist instead.")
  (safety_dist m))

(cl:ensure-generic-function 'lethal_cost_mult-val :lambda-list '(m))
(cl:defmethod lethal_cost_mult-val ((m <CheckPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:lethal_cost_mult-val is deprecated.  Use mbf_msgs-srv:lethal_cost_mult instead.")
  (lethal_cost_mult m))

(cl:ensure-generic-function 'inscrib_cost_mult-val :lambda-list '(m))
(cl:defmethod inscrib_cost_mult-val ((m <CheckPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:inscrib_cost_mult-val is deprecated.  Use mbf_msgs-srv:inscrib_cost_mult instead.")
  (inscrib_cost_mult m))

(cl:ensure-generic-function 'unknown_cost_mult-val :lambda-list '(m))
(cl:defmethod unknown_cost_mult-val ((m <CheckPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:unknown_cost_mult-val is deprecated.  Use mbf_msgs-srv:unknown_cost_mult instead.")
  (unknown_cost_mult m))

(cl:ensure-generic-function 'costmap-val :lambda-list '(m))
(cl:defmethod costmap-val ((m <CheckPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:costmap-val is deprecated.  Use mbf_msgs-srv:costmap instead.")
  (costmap m))

(cl:ensure-generic-function 'current_pose-val :lambda-list '(m))
(cl:defmethod current_pose-val ((m <CheckPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:current_pose-val is deprecated.  Use mbf_msgs-srv:current_pose instead.")
  (current_pose m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CheckPose-request>)))
    "Constants for message type '<CheckPose-request>"
  '((:LOCAL_COSTMAP . 1)
    (:GLOBAL_COSTMAP . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CheckPose-request)))
    "Constants for message type 'CheckPose-request"
  '((:LOCAL_COSTMAP . 1)
    (:GLOBAL_COSTMAP . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPose-request>) ostream)
  "Serializes a message object of type '<CheckPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'current_pose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPose-request>) istream)
  "Deserializes a message object of type '<CheckPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
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
    (cl:setf (cl:slot-value msg 'current_pose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPose-request>)))
  "Returns string type for a service object of type '<CheckPose-request>"
  "mbf_msgs/CheckPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPose-request)))
  "Returns string type for a service object of type 'CheckPose-request"
  "mbf_msgs/CheckPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPose-request>)))
  "Returns md5sum for a message object of type '<CheckPose-request>"
  "540ccc67006cf1c2c5502427e26c7f21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPose-request)))
  "Returns md5sum for a message object of type 'CheckPose-request"
  "540ccc67006cf1c2c5502427e26c7f21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPose-request>)))
  "Returns full string definition for message of type '<CheckPose-request>"
  (cl:format cl:nil "uint8                      LOCAL_COSTMAP  = 1~%uint8                      GLOBAL_COSTMAP = 2~%~%geometry_msgs/PoseStamped  pose              # the pose to be checked after transforming to costmap frame~%float32                    safety_dist       # minimum distance allowed to the closest obstacle~%float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)~%float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)~%float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)~%uint8                      costmap           # costmap in which to check the pose~%bool                       current_pose      # check current robot pose instead (ignores pose field)~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPose-request)))
  "Returns full string definition for message of type 'CheckPose-request"
  (cl:format cl:nil "uint8                      LOCAL_COSTMAP  = 1~%uint8                      GLOBAL_COSTMAP = 2~%~%geometry_msgs/PoseStamped  pose              # the pose to be checked after transforming to costmap frame~%float32                    safety_dist       # minimum distance allowed to the closest obstacle~%float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)~%float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)~%float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)~%uint8                      costmap           # costmap in which to check the pose~%bool                       current_pose      # check current robot pose instead (ignores pose field)~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPose-request
    (cl:cons ':pose (pose msg))
    (cl:cons ':safety_dist (safety_dist msg))
    (cl:cons ':lethal_cost_mult (lethal_cost_mult msg))
    (cl:cons ':inscrib_cost_mult (inscrib_cost_mult msg))
    (cl:cons ':unknown_cost_mult (unknown_cost_mult msg))
    (cl:cons ':costmap (costmap msg))
    (cl:cons ':current_pose (current_pose msg))
))
;//! \htmlinclude CheckPose-response.msg.html

(cl:defclass <CheckPose-response> (roslisp-msg-protocol:ros-message)
  ((state
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

(cl:defclass CheckPose-response (<CheckPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-srv:<CheckPose-response> is deprecated: use mbf_msgs-srv:CheckPose-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <CheckPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:state-val is deprecated.  Use mbf_msgs-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'cost-val :lambda-list '(m))
(cl:defmethod cost-val ((m <CheckPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:cost-val is deprecated.  Use mbf_msgs-srv:cost instead.")
  (cost m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CheckPose-response>)))
    "Constants for message type '<CheckPose-response>"
  '((:FREE . 0)
    (:INSCRIBED . 1)
    (:LETHAL . 2)
    (:UNKNOWN . 3)
    (:OUTSIDE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CheckPose-response)))
    "Constants for message type 'CheckPose-response"
  '((:FREE . 0)
    (:INSCRIBED . 1)
    (:LETHAL . 2)
    (:UNKNOWN . 3)
    (:OUTSIDE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPose-response>) ostream)
  "Serializes a message object of type '<CheckPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cost)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPose-response>) istream)
  "Deserializes a message object of type '<CheckPose-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cost)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPose-response>)))
  "Returns string type for a service object of type '<CheckPose-response>"
  "mbf_msgs/CheckPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPose-response)))
  "Returns string type for a service object of type 'CheckPose-response"
  "mbf_msgs/CheckPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPose-response>)))
  "Returns md5sum for a message object of type '<CheckPose-response>"
  "540ccc67006cf1c2c5502427e26c7f21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPose-response)))
  "Returns md5sum for a message object of type 'CheckPose-response"
  "540ccc67006cf1c2c5502427e26c7f21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPose-response>)))
  "Returns full string definition for message of type '<CheckPose-response>"
  (cl:format cl:nil "uint8                      FREE      =  0    # robot is completely in traversable space~%uint8                      INSCRIBED =  1    # robot is partially in inscribed space~%uint8                      LETHAL    =  2    # robot is partially in collision~%uint8                      UNKNOWN   =  3    # robot is partially in unknown space~%uint8                      OUTSIDE   =  4    # robot is partially outside the map~%~%uint8                      state             # pose state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE~%uint32                     cost              # total cost of all cells within footprint padded by safety_dist~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPose-response)))
  "Returns full string definition for message of type 'CheckPose-response"
  (cl:format cl:nil "uint8                      FREE      =  0    # robot is completely in traversable space~%uint8                      INSCRIBED =  1    # robot is partially in inscribed space~%uint8                      LETHAL    =  2    # robot is partially in collision~%uint8                      UNKNOWN   =  3    # robot is partially in unknown space~%uint8                      OUTSIDE   =  4    # robot is partially outside the map~%~%uint8                      state             # pose state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE~%uint32                     cost              # total cost of all cells within footprint padded by safety_dist~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPose-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPose-response
    (cl:cons ':state (state msg))
    (cl:cons ':cost (cost msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckPose)))
  'CheckPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckPose)))
  'CheckPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPose)))
  "Returns string type for a service object of type '<CheckPose>"
  "mbf_msgs/CheckPose")