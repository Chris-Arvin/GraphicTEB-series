; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude GetPathResult.msg.html

(cl:defclass <GetPathResult> (roslisp-msg-protocol:ros-message)
  ((outcome
    :reader outcome
    :initarg :outcome
    :type cl:integer
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path))
   (cost
    :reader cost
    :initarg :cost
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetPathResult (<GetPathResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPathResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPathResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<GetPathResult> is deprecated: use mbf_msgs-msg:GetPathResult instead.")))

(cl:ensure-generic-function 'outcome-val :lambda-list '(m))
(cl:defmethod outcome-val ((m <GetPathResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:outcome-val is deprecated.  Use mbf_msgs-msg:outcome instead.")
  (outcome m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GetPathResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:message-val is deprecated.  Use mbf_msgs-msg:message instead.")
  (message m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <GetPathResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:path-val is deprecated.  Use mbf_msgs-msg:path instead.")
  (path m))

(cl:ensure-generic-function 'cost-val :lambda-list '(m))
(cl:defmethod cost-val ((m <GetPathResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:cost-val is deprecated.  Use mbf_msgs-msg:cost instead.")
  (cost m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GetPathResult>)))
    "Constants for message type '<GetPathResult>"
  '((:SUCCESS . 0)
    (:FAILURE . 50)
    (:CANCELED . 51)
    (:INVALID_START . 52)
    (:INVALID_GOAL . 53)
    (:NO_PATH_FOUND . 54)
    (:PAT_EXCEEDED . 55)
    (:EMPTY_PATH . 56)
    (:TF_ERROR . 57)
    (:NOT_INITIALIZED . 58)
    (:INVALID_PLUGIN . 59)
    (:INTERNAL_ERROR . 60)
    (:OUT_OF_MAP . 61)
    (:MAP_ERROR . 62)
    (:STOPPED . 63))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GetPathResult)))
    "Constants for message type 'GetPathResult"
  '((:SUCCESS . 0)
    (:FAILURE . 50)
    (:CANCELED . 51)
    (:INVALID_START . 52)
    (:INVALID_GOAL . 53)
    (:NO_PATH_FOUND . 54)
    (:PAT_EXCEEDED . 55)
    (:EMPTY_PATH . 56)
    (:TF_ERROR . 57)
    (:NOT_INITIALIZED . 58)
    (:INVALID_PLUGIN . 59)
    (:INTERNAL_ERROR . 60)
    (:OUT_OF_MAP . 61)
    (:MAP_ERROR . 62)
    (:STOPPED . 63))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPathResult>) ostream)
  "Serializes a message object of type '<GetPathResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outcome)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'outcome)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'outcome)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'outcome)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPathResult>) istream)
  "Deserializes a message object of type '<GetPathResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outcome)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'outcome)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'outcome)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'outcome)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cost) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPathResult>)))
  "Returns string type for a message object of type '<GetPathResult>"
  "mbf_msgs/GetPathResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPathResult)))
  "Returns string type for a message object of type 'GetPathResult"
  "mbf_msgs/GetPathResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPathResult>)))
  "Returns md5sum for a message object of type '<GetPathResult>"
  "b737d51aec954f878a4cd57e83f5767c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPathResult)))
  "Returns md5sum for a message object of type 'GetPathResult"
  "b737d51aec954f878a4cd57e83f5767c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPathResult>)))
  "Returns full string definition for message of type '<GetPathResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS         = 0~%# 1..9 are reserved as plugin specific non-error results~%~%# Possible error codes:~%uint8 FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins~%uint8 CANCELED        = 51  # The action has been canceled by a action client~%uint8 INVALID_START   = 52  #~%uint8 INVALID_GOAL    = 53~%uint8 NO_PATH_FOUND   = 54~%uint8 PAT_EXCEEDED    = 55~%uint8 EMPTY_PATH      = 56~%uint8 TF_ERROR        = 57~%uint8 NOT_INITIALIZED = 58~%uint8 INVALID_PLUGIN  = 59~%uint8 INTERNAL_ERROR  = 60~%uint8 OUT_OF_MAP      = 61~%uint8 MAP_ERROR       = 62~%uint8 STOPPED         = 63  # The planner execution has been stopped rigorously.~%~%# 71..99 are reserved as plugin specific errors~%~%uint32 outcome~%string message~%~%nav_msgs/Path path~%~%float64 cost~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPathResult)))
  "Returns full string definition for message of type 'GetPathResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS         = 0~%# 1..9 are reserved as plugin specific non-error results~%~%# Possible error codes:~%uint8 FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins~%uint8 CANCELED        = 51  # The action has been canceled by a action client~%uint8 INVALID_START   = 52  #~%uint8 INVALID_GOAL    = 53~%uint8 NO_PATH_FOUND   = 54~%uint8 PAT_EXCEEDED    = 55~%uint8 EMPTY_PATH      = 56~%uint8 TF_ERROR        = 57~%uint8 NOT_INITIALIZED = 58~%uint8 INVALID_PLUGIN  = 59~%uint8 INTERNAL_ERROR  = 60~%uint8 OUT_OF_MAP      = 61~%uint8 MAP_ERROR       = 62~%uint8 STOPPED         = 63  # The planner execution has been stopped rigorously.~%~%# 71..99 are reserved as plugin specific errors~%~%uint32 outcome~%string message~%~%nav_msgs/Path path~%~%float64 cost~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPathResult>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'message))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPathResult>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPathResult
    (cl:cons ':outcome (outcome msg))
    (cl:cons ':message (message msg))
    (cl:cons ':path (path msg))
    (cl:cons ':cost (cost msg))
))
