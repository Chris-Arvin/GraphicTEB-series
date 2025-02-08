; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude ExePathGoal.msg.html

(cl:defclass <ExePathGoal> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path))
   (controller
    :reader controller
    :initarg :controller
    :type cl:string
    :initform "")
   (concurrency_slot
    :reader concurrency_slot
    :initarg :concurrency_slot
    :type cl:fixnum
    :initform 0)
   (tolerance_from_action
    :reader tolerance_from_action
    :initarg :tolerance_from_action
    :type cl:boolean
    :initform cl:nil)
   (dist_tolerance
    :reader dist_tolerance
    :initarg :dist_tolerance
    :type cl:float
    :initform 0.0)
   (angle_tolerance
    :reader angle_tolerance
    :initarg :angle_tolerance
    :type cl:float
    :initform 0.0))
)

(cl:defclass ExePathGoal (<ExePathGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExePathGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExePathGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<ExePathGoal> is deprecated: use mbf_msgs-msg:ExePathGoal instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <ExePathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:path-val is deprecated.  Use mbf_msgs-msg:path instead.")
  (path m))

(cl:ensure-generic-function 'controller-val :lambda-list '(m))
(cl:defmethod controller-val ((m <ExePathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:controller-val is deprecated.  Use mbf_msgs-msg:controller instead.")
  (controller m))

(cl:ensure-generic-function 'concurrency_slot-val :lambda-list '(m))
(cl:defmethod concurrency_slot-val ((m <ExePathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:concurrency_slot-val is deprecated.  Use mbf_msgs-msg:concurrency_slot instead.")
  (concurrency_slot m))

(cl:ensure-generic-function 'tolerance_from_action-val :lambda-list '(m))
(cl:defmethod tolerance_from_action-val ((m <ExePathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:tolerance_from_action-val is deprecated.  Use mbf_msgs-msg:tolerance_from_action instead.")
  (tolerance_from_action m))

(cl:ensure-generic-function 'dist_tolerance-val :lambda-list '(m))
(cl:defmethod dist_tolerance-val ((m <ExePathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:dist_tolerance-val is deprecated.  Use mbf_msgs-msg:dist_tolerance instead.")
  (dist_tolerance m))

(cl:ensure-generic-function 'angle_tolerance-val :lambda-list '(m))
(cl:defmethod angle_tolerance-val ((m <ExePathGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:angle_tolerance-val is deprecated.  Use mbf_msgs-msg:angle_tolerance instead.")
  (angle_tolerance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExePathGoal>) ostream)
  "Serializes a message object of type '<ExePathGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'concurrency_slot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tolerance_from_action) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist_tolerance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_tolerance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExePathGoal>) istream)
  "Deserializes a message object of type '<ExePathGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'controller) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'concurrency_slot)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance_from_action) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist_tolerance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_tolerance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExePathGoal>)))
  "Returns string type for a message object of type '<ExePathGoal>"
  "mbf_msgs/ExePathGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExePathGoal)))
  "Returns string type for a message object of type 'ExePathGoal"
  "mbf_msgs/ExePathGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExePathGoal>)))
  "Returns md5sum for a message object of type '<ExePathGoal>"
  "997d05ac3260fea4e2e2586ca47d2578")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExePathGoal)))
  "Returns md5sum for a message object of type 'ExePathGoal"
  "997d05ac3260fea4e2e2586ca47d2578")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExePathGoal>)))
  "Returns full string definition for message of type '<ExePathGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Follow the given path until completion or failure~%~%nav_msgs/Path path~%~%# Controller to use; defaults to the first one specified on \"controllers\" parameter~%string controller~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%# define goal tolerance for the action~%bool tolerance_from_action~%float32 dist_tolerance~%float32 angle_tolerance~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExePathGoal)))
  "Returns full string definition for message of type 'ExePathGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Follow the given path until completion or failure~%~%nav_msgs/Path path~%~%# Controller to use; defaults to the first one specified on \"controllers\" parameter~%string controller~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%# define goal tolerance for the action~%bool tolerance_from_action~%float32 dist_tolerance~%float32 angle_tolerance~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExePathGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     4 (cl:length (cl:slot-value msg 'controller))
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExePathGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'ExePathGoal
    (cl:cons ':path (path msg))
    (cl:cons ':controller (controller msg))
    (cl:cons ':concurrency_slot (concurrency_slot msg))
    (cl:cons ':tolerance_from_action (tolerance_from_action msg))
    (cl:cons ':dist_tolerance (dist_tolerance msg))
    (cl:cons ':angle_tolerance (angle_tolerance msg))
))
