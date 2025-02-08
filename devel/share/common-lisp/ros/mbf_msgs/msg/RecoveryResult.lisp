; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude RecoveryResult.msg.html

(cl:defclass <RecoveryResult> (roslisp-msg-protocol:ros-message)
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
   (used_plugin
    :reader used_plugin
    :initarg :used_plugin
    :type cl:string
    :initform ""))
)

(cl:defclass RecoveryResult (<RecoveryResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RecoveryResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RecoveryResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<RecoveryResult> is deprecated: use mbf_msgs-msg:RecoveryResult instead.")))

(cl:ensure-generic-function 'outcome-val :lambda-list '(m))
(cl:defmethod outcome-val ((m <RecoveryResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:outcome-val is deprecated.  Use mbf_msgs-msg:outcome instead.")
  (outcome m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <RecoveryResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:message-val is deprecated.  Use mbf_msgs-msg:message instead.")
  (message m))

(cl:ensure-generic-function 'used_plugin-val :lambda-list '(m))
(cl:defmethod used_plugin-val ((m <RecoveryResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:used_plugin-val is deprecated.  Use mbf_msgs-msg:used_plugin instead.")
  (used_plugin m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RecoveryResult>)))
    "Constants for message type '<RecoveryResult>"
  '((:SUCCESS . 0)
    (:FAILURE . 150)
    (:CANCELED . 151)
    (:PAT_EXCEEDED . 152)
    (:TF_ERROR . 153)
    (:NOT_INITIALIZED . 154)
    (:INVALID_PLUGIN . 155)
    (:INTERNAL_ERROR . 156)
    (:STOPPED . 157)
    (:IMPASSABLE . 158))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RecoveryResult)))
    "Constants for message type 'RecoveryResult"
  '((:SUCCESS . 0)
    (:FAILURE . 150)
    (:CANCELED . 151)
    (:PAT_EXCEEDED . 152)
    (:TF_ERROR . 153)
    (:NOT_INITIALIZED . 154)
    (:INVALID_PLUGIN . 155)
    (:INTERNAL_ERROR . 156)
    (:STOPPED . 157)
    (:IMPASSABLE . 158))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecoveryResult>) ostream)
  "Serializes a message object of type '<RecoveryResult>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'used_plugin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'used_plugin))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecoveryResult>) istream)
  "Deserializes a message object of type '<RecoveryResult>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'used_plugin) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'used_plugin) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RecoveryResult>)))
  "Returns string type for a message object of type '<RecoveryResult>"
  "mbf_msgs/RecoveryResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecoveryResult)))
  "Returns string type for a message object of type 'RecoveryResult"
  "mbf_msgs/RecoveryResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RecoveryResult>)))
  "Returns md5sum for a message object of type '<RecoveryResult>"
  "41d522f528f315af4a6c19e2fde7a3d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecoveryResult)))
  "Returns md5sum for a message object of type 'RecoveryResult"
  "41d522f528f315af4a6c19e2fde7a3d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecoveryResult>)))
  "Returns full string definition for message of type '<RecoveryResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS         = 0~%~%# Possible server codes:~%uint8 FAILURE         = 150~%uint8 CANCELED        = 151~%uint8 PAT_EXCEEDED    = 152~%uint8 TF_ERROR        = 153~%uint8 NOT_INITIALIZED = 154~%uint8 INVALID_PLUGIN  = 155~%uint8 INTERNAL_ERROR  = 156~%uint8 STOPPED         = 157  # The recovery behaviour execution has been stopped rigorously.~%uint8 IMPASSABLE      = 158  # Further execution would lead to a collision~%~%# 171..199 are reserved as plugin specific errors~%~%uint32 outcome~%string message~%string used_plugin~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecoveryResult)))
  "Returns full string definition for message of type 'RecoveryResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS         = 0~%~%# Possible server codes:~%uint8 FAILURE         = 150~%uint8 CANCELED        = 151~%uint8 PAT_EXCEEDED    = 152~%uint8 TF_ERROR        = 153~%uint8 NOT_INITIALIZED = 154~%uint8 INVALID_PLUGIN  = 155~%uint8 INTERNAL_ERROR  = 156~%uint8 STOPPED         = 157  # The recovery behaviour execution has been stopped rigorously.~%uint8 IMPASSABLE      = 158  # Further execution would lead to a collision~%~%# 171..199 are reserved as plugin specific errors~%~%uint32 outcome~%string message~%string used_plugin~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecoveryResult>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:length (cl:slot-value msg 'used_plugin))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecoveryResult>))
  "Converts a ROS message object to a list"
  (cl:list 'RecoveryResult
    (cl:cons ':outcome (outcome msg))
    (cl:cons ':message (message msg))
    (cl:cons ':used_plugin (used_plugin msg))
))
