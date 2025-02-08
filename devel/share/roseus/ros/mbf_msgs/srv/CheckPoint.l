;; Auto-generated. Do not edit!


(when (boundp 'mbf_msgs::CheckPoint)
  (if (not (find-package "MBF_MSGS"))
    (make-package "MBF_MSGS"))
  (shadow 'CheckPoint (find-package "MBF_MSGS")))
(unless (find-package "MBF_MSGS::CHECKPOINT")
  (make-package "MBF_MSGS::CHECKPOINT"))
(unless (find-package "MBF_MSGS::CHECKPOINTREQUEST")
  (make-package "MBF_MSGS::CHECKPOINTREQUEST"))
(unless (find-package "MBF_MSGS::CHECKPOINTRESPONSE")
  (make-package "MBF_MSGS::CHECKPOINTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(intern "*LOCAL_COSTMAP*" (find-package "MBF_MSGS::CHECKPOINTREQUEST"))
(shadow '*LOCAL_COSTMAP* (find-package "MBF_MSGS::CHECKPOINTREQUEST"))
(defconstant mbf_msgs::CheckPointRequest::*LOCAL_COSTMAP* 1)
(intern "*GLOBAL_COSTMAP*" (find-package "MBF_MSGS::CHECKPOINTREQUEST"))
(shadow '*GLOBAL_COSTMAP* (find-package "MBF_MSGS::CHECKPOINTREQUEST"))
(defconstant mbf_msgs::CheckPointRequest::*GLOBAL_COSTMAP* 2)

(defun mbf_msgs::CheckPointRequest-to-symbol (const)
  (cond
        ((= const 1) 'mbf_msgs::CheckPointRequest::*LOCAL_COSTMAP*)
        ((= const 2) 'mbf_msgs::CheckPointRequest::*GLOBAL_COSTMAP*)
        (t nil)))

(defclass mbf_msgs::CheckPointRequest
  :super ros::object
  :slots (_point _costmap ))

(defmethod mbf_msgs::CheckPointRequest
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::PointStamped :init))
    ((:costmap __costmap) 0)
    )
   (send-super :init)
   (setq _point __point)
   (setq _costmap (round __costmap))
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:costmap
   (&optional __costmap)
   (if __costmap (setq _costmap __costmap)) _costmap)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped _point
    (send _point :serialization-length)
    ;; uint8 _costmap
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped _point
       (send _point :serialize s)
     ;; uint8 _costmap
       (write-byte _costmap s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; uint8 _costmap
     (setq _costmap (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(intern "*FREE*" (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(shadow '*FREE* (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(defconstant mbf_msgs::CheckPointResponse::*FREE* 0)
(intern "*INSCRIBED*" (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(shadow '*INSCRIBED* (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(defconstant mbf_msgs::CheckPointResponse::*INSCRIBED* 1)
(intern "*LETHAL*" (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(shadow '*LETHAL* (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(defconstant mbf_msgs::CheckPointResponse::*LETHAL* 2)
(intern "*UNKNOWN*" (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(shadow '*UNKNOWN* (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(defconstant mbf_msgs::CheckPointResponse::*UNKNOWN* 3)
(intern "*OUTSIDE*" (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(shadow '*OUTSIDE* (find-package "MBF_MSGS::CHECKPOINTRESPONSE"))
(defconstant mbf_msgs::CheckPointResponse::*OUTSIDE* 4)

(defun mbf_msgs::CheckPointResponse-to-symbol (const)
  (cond
        ((= const 0) 'mbf_msgs::CheckPointResponse::*FREE*)
        ((= const 1) 'mbf_msgs::CheckPointResponse::*INSCRIBED*)
        ((= const 2) 'mbf_msgs::CheckPointResponse::*LETHAL*)
        ((= const 3) 'mbf_msgs::CheckPointResponse::*UNKNOWN*)
        ((= const 4) 'mbf_msgs::CheckPointResponse::*OUTSIDE*)
        (t nil)))

(defclass mbf_msgs::CheckPointResponse
  :super ros::object
  :slots (_state _cost ))

(defmethod mbf_msgs::CheckPointResponse
  (:init
   (&key
    ((:state __state) 0)
    ((:cost __cost) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   (setq _cost (round __cost))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ;; uint32 _cost
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;; uint32 _cost
       (write-long _cost s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _cost
     (setq _cost (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mbf_msgs::CheckPoint
  :super ros::object
  :slots ())

(setf (get mbf_msgs::CheckPoint :md5sum-) "098ffffe56a4f2f59d33aa09df0749c1")
(setf (get mbf_msgs::CheckPoint :datatype-) "mbf_msgs/CheckPoint")
(setf (get mbf_msgs::CheckPoint :request) mbf_msgs::CheckPointRequest)
(setf (get mbf_msgs::CheckPoint :response) mbf_msgs::CheckPointResponse)

(defmethod mbf_msgs::CheckPointRequest
  (:response () (instance mbf_msgs::CheckPointResponse :init)))

(setf (get mbf_msgs::CheckPointRequest :md5sum-) "098ffffe56a4f2f59d33aa09df0749c1")
(setf (get mbf_msgs::CheckPointRequest :datatype-) "mbf_msgs/CheckPointRequest")
(setf (get mbf_msgs::CheckPointRequest :definition-)
      "uint8                      LOCAL_COSTMAP  = 1
uint8                      GLOBAL_COSTMAP = 2

geometry_msgs/PointStamped point             # the point to be checked after transforming to costmap frame
uint8                      costmap           # costmap in which to check the point

================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
uint8                      FREE      =  0    # point is in traversable space
uint8                      INSCRIBED =  1    # point is in inscribed space
uint8                      LETHAL    =  2    # point is in collision
uint8                      UNKNOWN   =  3    # point is in unknown space
uint8                      OUTSIDE   =  4    # point is outside the map

uint8                      state             # point state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE
uint32                     cost              # cost of the cell at point

")

(setf (get mbf_msgs::CheckPointResponse :md5sum-) "098ffffe56a4f2f59d33aa09df0749c1")
(setf (get mbf_msgs::CheckPointResponse :datatype-) "mbf_msgs/CheckPointResponse")
(setf (get mbf_msgs::CheckPointResponse :definition-)
      "uint8                      LOCAL_COSTMAP  = 1
uint8                      GLOBAL_COSTMAP = 2

geometry_msgs/PointStamped point             # the point to be checked after transforming to costmap frame
uint8                      costmap           # costmap in which to check the point

================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
uint8                      FREE      =  0    # point is in traversable space
uint8                      INSCRIBED =  1    # point is in inscribed space
uint8                      LETHAL    =  2    # point is in collision
uint8                      UNKNOWN   =  3    # point is in unknown space
uint8                      OUTSIDE   =  4    # point is outside the map

uint8                      state             # point state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE
uint32                     cost              # cost of the cell at point

")



(provide :mbf_msgs/CheckPoint "098ffffe56a4f2f59d33aa09df0749c1")


