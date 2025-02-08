; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude GetPathGoal.msg.html

(cl:defclass <GetPathGoal> (roslisp-msg-protocol:ros-message)
  ((use_start_pose
    :reader use_start_pose
    :initarg :use_start_pose
    :type cl:boolean
    :initform cl:nil)
   (start_pose
    :reader start_pose
    :initarg :start_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (tolerance
    :reader tolerance
    :initarg :tolerance
    :type cl:float
    :initform 0.0)
   (planner
    :reader planner
    :initarg :planner
    :type cl:string
    :initform "")
   (concurrency_slot
    :reader concurrency_slot
    :initarg :concurrency_slot
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetPathGoal (<GetPathGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPathGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPathGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<GetPathGoal> is deprecated: use mbf_msgs-msg:GetPathGoal instead.")))

(cl:ensure-generic-function 'use_start_pose-val :lambda-list '(m))
(cl:defmethod use_start_pose-val ((m <GetPathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:use_start_pose-val is deprecated.  Use mbf_msgs-msg:use_start_pose instead.")
  (use_start_pose m))

(cl:ensure-generic-function 'start_pose-val :lambda-list '(m))
(cl:defmethod start_pose-val ((m <GetPathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:start_pose-val is deprecated.  Use mbf_msgs-msg:start_pose instead.")
  (start_pose m))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <GetPathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:target_pose-val is deprecated.  Use mbf_msgs-msg:target_pose instead.")
  (target_pose m))

(cl:ensure-generic-function 'tolerance-val :lambda-list '(m))
(cl:defmethod tolerance-val ((m <GetPathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:tolerance-val is deprecated.  Use mbf_msgs-msg:tolerance instead.")
  (tolerance m))

(cl:ensure-generic-function 'planner-val :lambda-list '(m))
(cl:defmethod planner-val ((m <GetPathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:planner-val is deprecated.  Use mbf_msgs-msg:planner instead.")
  (planner m))

(cl:ensure-generic-function 'concurrency_slot-val :lambda-list '(m))
(cl:defmethod concurrency_slot-val ((m <GetPathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:concurrency_slot-val is deprecated.  Use mbf_msgs-msg:concurrency_slot instead.")
  (concurrency_slot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPathGoal>) ostream)
  "Serializes a message object of type '<GetPathGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_start_pose) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tolerance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'planner))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'planner))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'concurrency_slot)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPathGoal>) istream)
  "Deserializes a message object of type '<GetPathGoal>"
    (cl:setf (cl:slot-value msg 'use_start_pose) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planner) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'planner) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'concurrency_slot)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPathGoal>)))
  "Returns string type for a message object of type '<GetPathGoal>"
  "mbf_msgs/GetPathGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPathGoal)))
  "Returns string type for a message object of type 'GetPathGoal"
  "mbf_msgs/GetPathGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPathGoal>)))
  "Returns md5sum for a message object of type '<GetPathGoal>"
  "301d9f5ec2f8f08d1d4e16663a6d2c5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPathGoal)))
  "Returns md5sum for a message object of type 'GetPathGoal"
  "301d9f5ec2f8f08d1d4e16663a6d2c5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPathGoal>)))
  "Returns full string definition for message of type '<GetPathGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Get a path from start_pose or current position to the target pose~%~%# Use start_pose or current position as the beginning of the path~%bool use_start_pose~%~%# The start pose for the path; optional, used if use_start_pose is true~%geometry_msgs/PoseStamped start_pose~%~%# The pose to achieve with the path~%geometry_msgs/PoseStamped target_pose~%~%# If the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing~%float64 tolerance~%~%# Planner to use; defaults to the first one specified on \"planners\" parameter~%string planner~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPathGoal)))
  "Returns full string definition for message of type 'GetPathGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Get a path from start_pose or current position to the target pose~%~%# Use start_pose or current position as the beginning of the path~%bool use_start_pose~%~%# The start pose for the path; optional, used if use_start_pose is true~%geometry_msgs/PoseStamped start_pose~%~%# The pose to achieve with the path~%geometry_msgs/PoseStamped target_pose~%~%# If the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing~%float64 tolerance~%~%# Planner to use; defaults to the first one specified on \"planners\" parameter~%string planner~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPathGoal>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
     8
     4 (cl:length (cl:slot-value msg 'planner))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPathGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPathGoal
    (cl:cons ':use_start_pose (use_start_pose msg))
    (cl:cons ':start_pose (start_pose msg))
    (cl:cons ':target_pose (target_pose msg))
    (cl:cons ':tolerance (tolerance msg))
    (cl:cons ':planner (planner msg))
    (cl:cons ':concurrency_slot (concurrency_slot msg))
))
