;; Auto-generated. Do not edit!


(when (boundp 'mbf_msgs::CheckPose)
  (if (not (find-package "MBF_MSGS"))
    (make-package "MBF_MSGS"))
  (shadow 'CheckPose (find-package "MBF_MSGS")))
(unless (find-package "MBF_MSGS::CHECKPOSE")
  (make-package "MBF_MSGS::CHECKPOSE"))
(unless (find-package "MBF_MSGS::CHECKPOSEREQUEST")
  (make-package "MBF_MSGS::CHECKPOSEREQUEST"))
(unless (find-package "MBF_MSGS::CHECKPOSERESPONSE")
  (make-package "MBF_MSGS::CHECKPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(intern "*LOCAL_COSTMAP*" (find-package "MBF_MSGS::CHECKPOSEREQUEST"))
(shadow '*LOCAL_COSTMAP* (find-package "MBF_MSGS::CHECKPOSEREQUEST"))
(defconstant mbf_msgs::CheckPoseRequest::*LOCAL_COSTMAP* 1)
(intern "*GLOBAL_COSTMAP*" (find-package "MBF_MSGS::CHECKPOSEREQUEST"))
(shadow '*GLOBAL_COSTMAP* (find-package "MBF_MSGS::CHECKPOSEREQUEST"))
(defconstant mbf_msgs::CheckPoseRequest::*GLOBAL_COSTMAP* 2)

(defun mbf_msgs::CheckPoseRequest-to-symbol (const)
  (cond
        ((= const 1) 'mbf_msgs::CheckPoseRequest::*LOCAL_COSTMAP*)
        ((= const 2) 'mbf_msgs::CheckPoseRequest::*GLOBAL_COSTMAP*)
        (t nil)))

(defclass mbf_msgs::CheckPoseRequest
  :super ros::object
  :slots (_pose _safety_dist _lethal_cost_mult _inscrib_cost_mult _unknown_cost_mult _costmap _current_pose ))

(defmethod mbf_msgs::CheckPoseRequest
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:safety_dist __safety_dist) 0.0)
    ((:lethal_cost_mult __lethal_cost_mult) 0.0)
    ((:inscrib_cost_mult __inscrib_cost_mult) 0.0)
    ((:unknown_cost_mult __unknown_cost_mult) 0.0)
    ((:costmap __costmap) 0)
    ((:current_pose __current_pose) nil)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _safety_dist (float __safety_dist))
   (setq _lethal_cost_mult (float __lethal_cost_mult))
   (setq _inscrib_cost_mult (float __inscrib_cost_mult))
   (setq _unknown_cost_mult (float __unknown_cost_mult))
   (setq _costmap (round __costmap))
   (setq _current_pose __current_pose)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:safety_dist
   (&optional __safety_dist)
   (if __safety_dist (setq _safety_dist __safety_dist)) _safety_dist)
  (:lethal_cost_mult
   (&optional __lethal_cost_mult)
   (if __lethal_cost_mult (setq _lethal_cost_mult __lethal_cost_mult)) _lethal_cost_mult)
  (:inscrib_cost_mult
   (&optional __inscrib_cost_mult)
   (if __inscrib_cost_mult (setq _inscrib_cost_mult __inscrib_cost_mult)) _inscrib_cost_mult)
  (:unknown_cost_mult
   (&optional __unknown_cost_mult)
   (if __unknown_cost_mult (setq _unknown_cost_mult __unknown_cost_mult)) _unknown_cost_mult)
  (:costmap
   (&optional __costmap)
   (if __costmap (setq _costmap __costmap)) _costmap)
  (:current_pose
   (&optional (__current_pose :null))
   (if (not (eq __current_pose :null)) (setq _current_pose __current_pose)) _current_pose)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; float32 _safety_dist
    4
    ;; float32 _lethal_cost_mult
    4
    ;; float32 _inscrib_cost_mult
    4
    ;; float32 _unknown_cost_mult
    4
    ;; uint8 _costmap
    1
    ;; bool _current_pose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; float32 _safety_dist
       (sys::poke _safety_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lethal_cost_mult
       (sys::poke _lethal_cost_mult (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _inscrib_cost_mult
       (sys::poke _inscrib_cost_mult (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _unknown_cost_mult
       (sys::poke _unknown_cost_mult (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _costmap
       (write-byte _costmap s)
     ;; bool _current_pose
       (if _current_pose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float32 _safety_dist
     (setq _safety_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lethal_cost_mult
     (setq _lethal_cost_mult (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _inscrib_cost_mult
     (setq _inscrib_cost_mult (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _unknown_cost_mult
     (setq _unknown_cost_mult (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _costmap
     (setq _costmap (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _current_pose
     (setq _current_pose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(intern "*FREE*" (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(shadow '*FREE* (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(defconstant mbf_msgs::CheckPoseResponse::*FREE* 0)
(intern "*INSCRIBED*" (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(shadow '*INSCRIBED* (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(defconstant mbf_msgs::CheckPoseResponse::*INSCRIBED* 1)
(intern "*LETHAL*" (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(shadow '*LETHAL* (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(defconstant mbf_msgs::CheckPoseResponse::*LETHAL* 2)
(intern "*UNKNOWN*" (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(shadow '*UNKNOWN* (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(defconstant mbf_msgs::CheckPoseResponse::*UNKNOWN* 3)
(intern "*OUTSIDE*" (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(shadow '*OUTSIDE* (find-package "MBF_MSGS::CHECKPOSERESPONSE"))
(defconstant mbf_msgs::CheckPoseResponse::*OUTSIDE* 4)

(defun mbf_msgs::CheckPoseResponse-to-symbol (const)
  (cond
        ((= const 0) 'mbf_msgs::CheckPoseResponse::*FREE*)
        ((= const 1) 'mbf_msgs::CheckPoseResponse::*INSCRIBED*)
        ((= const 2) 'mbf_msgs::CheckPoseResponse::*LETHAL*)
        ((= const 3) 'mbf_msgs::CheckPoseResponse::*UNKNOWN*)
        ((= const 4) 'mbf_msgs::CheckPoseResponse::*OUTSIDE*)
        (t nil)))

(defclass mbf_msgs::CheckPoseResponse
  :super ros::object
  :slots (_state _cost ))

(defmethod mbf_msgs::CheckPoseResponse
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

(defclass mbf_msgs::CheckPose
  :super ros::object
  :slots ())

(setf (get mbf_msgs::CheckPose :md5sum-) "540ccc67006cf1c2c5502427e26c7f21")
(setf (get mbf_msgs::CheckPose :datatype-) "mbf_msgs/CheckPose")
(setf (get mbf_msgs::CheckPose :request) mbf_msgs::CheckPoseRequest)
(setf (get mbf_msgs::CheckPose :response) mbf_msgs::CheckPoseResponse)

(defmethod mbf_msgs::CheckPoseRequest
  (:response () (instance mbf_msgs::CheckPoseResponse :init)))

(setf (get mbf_msgs::CheckPoseRequest :md5sum-) "540ccc67006cf1c2c5502427e26c7f21")
(setf (get mbf_msgs::CheckPoseRequest :datatype-) "mbf_msgs/CheckPoseRequest")
(setf (get mbf_msgs::CheckPoseRequest :definition-)
      "uint8                      LOCAL_COSTMAP  = 1
uint8                      GLOBAL_COSTMAP = 2

geometry_msgs/PoseStamped  pose              # the pose to be checked after transforming to costmap frame
float32                    safety_dist       # minimum distance allowed to the closest obstacle
float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)
float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)
float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)
uint8                      costmap           # costmap in which to check the pose
bool                       current_pose      # check current robot pose instead (ignores pose field)

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
uint8                      FREE      =  0    # robot is completely in traversable space
uint8                      INSCRIBED =  1    # robot is partially in inscribed space
uint8                      LETHAL    =  2    # robot is partially in collision
uint8                      UNKNOWN   =  3    # robot is partially in unknown space
uint8                      OUTSIDE   =  4    # robot is partially outside the map

uint8                      state             # pose state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE
uint32                     cost              # total cost of all cells within footprint padded by safety_dist

")

(setf (get mbf_msgs::CheckPoseResponse :md5sum-) "540ccc67006cf1c2c5502427e26c7f21")
(setf (get mbf_msgs::CheckPoseResponse :datatype-) "mbf_msgs/CheckPoseResponse")
(setf (get mbf_msgs::CheckPoseResponse :definition-)
      "uint8                      LOCAL_COSTMAP  = 1
uint8                      GLOBAL_COSTMAP = 2

geometry_msgs/PoseStamped  pose              # the pose to be checked after transforming to costmap frame
float32                    safety_dist       # minimum distance allowed to the closest obstacle
float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)
float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)
float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)
uint8                      costmap           # costmap in which to check the pose
bool                       current_pose      # check current robot pose instead (ignores pose field)

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
uint8                      FREE      =  0    # robot is completely in traversable space
uint8                      INSCRIBED =  1    # robot is partially in inscribed space
uint8                      LETHAL    =  2    # robot is partially in collision
uint8                      UNKNOWN   =  3    # robot is partially in unknown space
uint8                      OUTSIDE   =  4    # robot is partially outside the map

uint8                      state             # pose state: FREE, INFLATED, LETHAL, UNKNOWN or OUTSIDE
uint32                     cost              # total cost of all cells within footprint padded by safety_dist

")



(provide :mbf_msgs/CheckPose "540ccc67006cf1c2c5502427e26c7f21")


