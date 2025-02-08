; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude MoveBaseResult.msg.html

(cl:defclass <MoveBaseResult> (roslisp-msg-protocol:ros-message)
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
   (dist_to_goal
    :reader dist_to_goal
    :initarg :dist_to_goal
    :type cl:float
    :initform 0.0)
   (angle_to_goal
    :reader angle_to_goal
    :initarg :angle_to_goal
    :type cl:float
    :initform 0.0)
   (final_pose
    :reader final_pose
    :initarg :final_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass MoveBaseResult (<MoveBaseResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveBaseResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveBaseResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<MoveBaseResult> is deprecated: use mbf_msgs-msg:MoveBaseResult instead.")))

(cl:ensure-generic-function 'outcome-val :lambda-list '(m))
(cl:defmethod outcome-val ((m <MoveBaseResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:outcome-val is deprecated.  Use mbf_msgs-msg:outcome instead.")
  (outcome m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <MoveBaseResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:message-val is deprecated.  Use mbf_msgs-msg:message instead.")
  (message m))

(cl:ensure-generic-function 'dist_to_goal-val :lambda-list '(m))
(cl:defmethod dist_to_goal-val ((m <MoveBaseResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:dist_to_goal-val is deprecated.  Use mbf_msgs-msg:dist_to_goal instead.")
  (dist_to_goal m))

(cl:ensure-generic-function 'angle_to_goal-val :lambda-list '(m))
(cl:defmethod angle_to_goal-val ((m <MoveBaseResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:angle_to_goal-val is deprecated.  Use mbf_msgs-msg:angle_to_goal instead.")
  (angle_to_goal m))

(cl:ensure-generic-function 'final_pose-val :lambda-list '(m))
(cl:defmethod final_pose-val ((m <MoveBaseResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:final_pose-val is deprecated.  Use mbf_msgs-msg:final_pose instead.")
  (final_pose m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MoveBaseResult>)))
    "Constants for message type '<MoveBaseResult>"
  '((:SUCCESS . 0)
    (:FAILURE . 10)
    (:CANCELED . 11)
    (:COLLISION . 12)
    (:OSCILLATION . 13)
    (:START_BLOCKED . 14)
    (:GOAL_BLOCKED . 15)
    (:TF_ERROR . 16)
    (:INTERNAL_ERROR . 17)
    (:PLAN_FAILURE . 50)
    (:CTRL_FAILURE . 100))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MoveBaseResult)))
    "Constants for message type 'MoveBaseResult"
  '((:SUCCESS . 0)
    (:FAILURE . 10)
    (:CANCELED . 11)
    (:COLLISION . 12)
    (:OSCILLATION . 13)
    (:START_BLOCKED . 14)
    (:GOAL_BLOCKED . 15)
    (:TF_ERROR . 16)
    (:INTERNAL_ERROR . 17)
    (:PLAN_FAILURE . 50)
    (:CTRL_FAILURE . 100))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveBaseResult>) ostream)
  "Serializes a message object of type '<MoveBaseResult>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist_to_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_to_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'final_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveBaseResult>) istream)
  "Deserializes a message object of type '<MoveBaseResult>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist_to_goal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_to_goal) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'final_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveBaseResult>)))
  "Returns string type for a message object of type '<MoveBaseResult>"
  "mbf_msgs/MoveBaseResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveBaseResult)))
  "Returns string type for a message object of type 'MoveBaseResult"
  "mbf_msgs/MoveBaseResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveBaseResult>)))
  "Returns md5sum for a message object of type '<MoveBaseResult>"
  "c65d301ffa20590244253c6a99c37c5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveBaseResult)))
  "Returns md5sum for a message object of type 'MoveBaseResult"
  "c65d301ffa20590244253c6a99c37c5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveBaseResult>)))
  "Returns full string definition for message of type '<MoveBaseResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS        = 0~%~%# Predefined general error codes:~%uint8 FAILURE        = 10~%uint8 CANCELED       = 11~%uint8 COLLISION      = 12~%uint8 OSCILLATION    = 13~%uint8 START_BLOCKED  = 14~%uint8 GOAL_BLOCKED   = 15~%uint8 TF_ERROR       = 16~%uint8 INTERNAL_ERROR = 17~%# 21..49 are reserved for future general error codes~%~%# Planning/controlling failures:~%uint8 PLAN_FAILURE   = 50~%# 51..99 are reserved as planner specific errors~%~%uint8 CTRL_FAILURE   = 100~%# 101..149 are reserved as controller specific errors~%~%uint32 outcome~%string message~%~%# Configuration upon action completion~%float32 dist_to_goal~%float32 angle_to_goal~%geometry_msgs/PoseStamped final_pose~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveBaseResult)))
  "Returns full string definition for message of type 'MoveBaseResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS        = 0~%~%# Predefined general error codes:~%uint8 FAILURE        = 10~%uint8 CANCELED       = 11~%uint8 COLLISION      = 12~%uint8 OSCILLATION    = 13~%uint8 START_BLOCKED  = 14~%uint8 GOAL_BLOCKED   = 15~%uint8 TF_ERROR       = 16~%uint8 INTERNAL_ERROR = 17~%# 21..49 are reserved for future general error codes~%~%# Planning/controlling failures:~%uint8 PLAN_FAILURE   = 50~%# 51..99 are reserved as planner specific errors~%~%uint8 CTRL_FAILURE   = 100~%# 101..149 are reserved as controller specific errors~%~%uint32 outcome~%string message~%~%# Configuration upon action completion~%float32 dist_to_goal~%float32 angle_to_goal~%geometry_msgs/PoseStamped final_pose~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveBaseResult>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'message))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'final_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveBaseResult>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveBaseResult
    (cl:cons ':outcome (outcome msg))
    (cl:cons ':message (message msg))
    (cl:cons ':dist_to_goal (dist_to_goal msg))
    (cl:cons ':angle_to_goal (angle_to_goal msg))
    (cl:cons ':final_pose (final_pose msg))
))
