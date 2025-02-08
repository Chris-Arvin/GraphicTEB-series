; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-srv)


;//! \htmlinclude CheckPoint-request.msg.html

(cl:defclass <CheckPoint-request> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (costmap
    :reader costmap
    :initarg :costmap
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CheckPoint-request (<CheckPoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-srv:<CheckPoint-request> is deprecated: use mbf_msgs-srv:CheckPoint-request instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <CheckPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:point-val is deprecated.  Use mbf_msgs-srv:point instead.")
  (point m))

(cl:ensure-generic-function 'costmap-val :lambda-list '(m))
(cl:defmethod costmap-val ((m <CheckPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:costmap-val is deprecated.  Use mbf_msgs-srv:costmap instead.")
  (costmap m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CheckPoint-request>)))
    "Constants for message type '<CheckPoint-request>"
  '((:LOCAL_COSTMAP . 1)
    (:GLOBAL_COSTMAP . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CheckPoint-request)))
    "Constants for message type 'CheckPoint-request"
  '((:LOCAL_COSTMAP . 1)
    (:GLOBAL_COSTMAP . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPoint-request>) ostream)
  "Serializes a message object of type '<CheckPoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'costmap)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPoint-request>) istream)
  "Deserializes a message object of type '<CheckPoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'costmap)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPoint-request>)))
  "Returns string type for a service object of type '<CheckPoint-request>"
  "mbf_msgs/CheckPointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPoint-request)))
  "Returns string type for a service object of type 'CheckPoint-request"
  "mbf_msgs/CheckPointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPoint-request>)))
  "Returns md5sum for a message object of type '<CheckPoint-request>"
  "098ffffe56a4f2f59d33aa09df0749c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPoint-request)))
  "Returns md5sum for a message object of type 'CheckPoint-request"
  "098ffffe56a4f2f59d33aa09df0749c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPoint-request>)))
  "Returns full string definition for message of type '<CheckPoint-request>"
  (cl:format cl:nil "uint8                      LOCAL_COSTMAP  = 1~%uint8                      GLOBAL_COSTMAP = 2~%~%geometry_msgs/PointStamped point             # the point to be checked after transforming to costmap frame~%uint8                      costmap           # costmap in which to check the point~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPoint-request)))
  "Returns full string definition for message of type 'CheckPoint-request"
  (cl:format cl:nil "uint8                      LOCAL_COSTMAP  = 1~%uint8                      GLOBAL_COSTMAP = 2~%~%geometry_msgs/PointStamped point             # the point to be checked after transforming to costmap frame~%uint8                      costmap           # costmap in which to check the point~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPoint-request
    (cl:cons ':point (point msg))
    (cl:cons ':costmap (costmap msg))
))
;//! \htmlinclude CheckPoint-response.msg.html

(cl:defclass <CheckPoint-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass CheckPoint-response (<CheckPoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-srv:<CheckPoint-response> is deprecated: use mbf_msgs-srv:CheckPoint-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <CheckPoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:state-val is deprecated.  Use mbf_msgs-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'cost-val :lambda-list '(m))
(cl:defmethod cost-val ((m <CheckPoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-srv:cost-val is deprecated.  Use mbf_msgs-srv:cost instead.")
  (cost m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CheckPoint-response>)))
    "Constants for message type '<CheckPoint-response>"
  '((:FREE . 0)
    (:INSCRIBED . 1)
    (:LETHAL . 2)
    (:UNKNOWN . 3)
    (:OUTSIDE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CheckPoint-response)))
    "Constants for message type 'CheckPoint-response"
  '((:FREE . 0)
    (:INSCRIBED . 1)
    (:LETHAL . 2)
    (:UNKNOWN . 3)
    (:OUTSIDE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPoint-response>) ostream)
  "Serializes a message object of type '<CheckPoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cost)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cost)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPoint-response>) istream)
  "Deserializes a message object of type '<CheckPoint-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cost)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cost)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPoint-response>)))
  "Returns string type for a service object of type '<CheckPoint-response>"
  "mbf_msgs/CheckPointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPoint-response)))
  "Returns string type for a service object of type 'CheckPoint-response"
  "mbf_msgs/CheckPointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPoint-response>)))
  "Returns md5sum for a message object of type '<CheckPoint-response>"
  "098ffffe56a4f2f59d33aa09df0749c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPoint-response)))
  "Returns md5sum for a message object of type 'CheckPoint-response"
  "098ffffe56a4f2f59d33aa09df0749c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPoint-response>)))
  "Returns full string definition for message of type '<CheckPoint-response>"
  (cl:format cl:nil "uint8                      FREE      =  0    # point is in traversable space~%uint8                      INSCRIBED =  1    # point is in inscribed space~%uint8                      LETHAL    =  2    # point is in collision~%uint8                      UNKNOWN   =  3    # point is in unknown space~%uint8                      OUTSIDE   =  4    # point is outside the map~%~%uint8                      state             # point state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE~%uint32                     cost              # cost of the cell at point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPoint-response)))
  "Returns full string definition for message of type 'CheckPoint-response"
  (cl:format cl:nil "uint8                      FREE      =  0    # point is in traversable space~%uint8                      INSCRIBED =  1    # point is in inscribed space~%uint8                      LETHAL    =  2    # point is in collision~%uint8                      UNKNOWN   =  3    # point is in unknown space~%uint8                      OUTSIDE   =  4    # point is outside the map~%~%uint8                      state             # point state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE~%uint32                     cost              # cost of the cell at point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPoint-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPoint-response
    (cl:cons ':state (state msg))
    (cl:cons ':cost (cost msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckPoint)))
  'CheckPoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckPoint)))
  'CheckPoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPoint)))
  "Returns string type for a service object of type '<CheckPoint>"
  "mbf_msgs/CheckPoint")