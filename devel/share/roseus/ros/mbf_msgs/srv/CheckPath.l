;; Auto-generated. Do not edit!


(when (boundp 'mbf_msgs::CheckPath)
  (if (not (find-package "MBF_MSGS"))
    (make-package "MBF_MSGS"))
  (shadow 'CheckPath (find-package "MBF_MSGS")))
(unless (find-package "MBF_MSGS::CHECKPATH")
  (make-package "MBF_MSGS::CHECKPATH"))
(unless (find-package "MBF_MSGS::CHECKPATHREQUEST")
  (make-package "MBF_MSGS::CHECKPATHREQUEST"))
(unless (find-package "MBF_MSGS::CHECKPATHRESPONSE")
  (make-package "MBF_MSGS::CHECKPATHRESPONSE"))

(in-package "ROS")

(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))




(intern "*LOCAL_COSTMAP*" (find-package "MBF_MSGS::CHECKPATHREQUEST"))
(shadow '*LOCAL_COSTMAP* (find-package "MBF_MSGS::CHECKPATHREQUEST"))
(defconstant mbf_msgs::CheckPathRequest::*LOCAL_COSTMAP* 1)
(intern "*GLOBAL_COSTMAP*" (find-package "MBF_MSGS::CHECKPATHREQUEST"))
(shadow '*GLOBAL_COSTMAP* (find-package "MBF_MSGS::CHECKPATHREQUEST"))
(defconstant mbf_msgs::CheckPathRequest::*GLOBAL_COSTMAP* 2)

(defun mbf_msgs::CheckPathRequest-to-symbol (const)
  (cond
        ((= const 1) 'mbf_msgs::CheckPathRequest::*LOCAL_COSTMAP*)
        ((= const 2) 'mbf_msgs::CheckPathRequest::*GLOBAL_COSTMAP*)
        (t nil)))

(defclass mbf_msgs::CheckPathRequest
  :super ros::object
  :slots (_path _safety_dist _lethal_cost_mult _inscrib_cost_mult _unknown_cost_mult _costmap _return_on _skip_poses _path_cells_only ))

(defmethod mbf_msgs::CheckPathRequest
  (:init
   (&key
    ((:path __path) (instance nav_msgs::Path :init))
    ((:safety_dist __safety_dist) 0.0)
    ((:lethal_cost_mult __lethal_cost_mult) 0.0)
    ((:inscrib_cost_mult __inscrib_cost_mult) 0.0)
    ((:unknown_cost_mult __unknown_cost_mult) 0.0)
    ((:costmap __costmap) 0)
    ((:return_on __return_on) 0)
    ((:skip_poses __skip_poses) 0)
    ((:path_cells_only __path_cells_only) nil)
    )
   (send-super :init)
   (setq _path __path)
   (setq _safety_dist (float __safety_dist))
   (setq _lethal_cost_mult (float __lethal_cost_mult))
   (setq _inscrib_cost_mult (float __inscrib_cost_mult))
   (setq _unknown_cost_mult (float __unknown_cost_mult))
   (setq _costmap (round __costmap))
   (setq _return_on (round __return_on))
   (setq _skip_poses (round __skip_poses))
   (setq _path_cells_only __path_cells_only)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
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
  (:return_on
   (&optional __return_on)
   (if __return_on (setq _return_on __return_on)) _return_on)
  (:skip_poses
   (&optional __skip_poses)
   (if __skip_poses (setq _skip_poses __skip_poses)) _skip_poses)
  (:path_cells_only
   (&optional (__path_cells_only :null))
   (if (not (eq __path_cells_only :null)) (setq _path_cells_only __path_cells_only)) _path_cells_only)
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
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
    ;; uint8 _return_on
    1
    ;; uint8 _skip_poses
    1
    ;; bool _path_cells_only
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _path
       (send _path :serialize s)
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
     ;; uint8 _return_on
       (write-byte _return_on s)
     ;; uint8 _skip_poses
       (write-byte _skip_poses s)
     ;; bool _path_cells_only
       (if _path_cells_only (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
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
   ;; uint8 _return_on
     (setq _return_on (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _skip_poses
     (setq _skip_poses (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _path_cells_only
     (setq _path_cells_only (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(intern "*FREE*" (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(shadow '*FREE* (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(defconstant mbf_msgs::CheckPathResponse::*FREE* 0)
(intern "*INSCRIBED*" (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(shadow '*INSCRIBED* (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(defconstant mbf_msgs::CheckPathResponse::*INSCRIBED* 1)
(intern "*LETHAL*" (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(shadow '*LETHAL* (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(defconstant mbf_msgs::CheckPathResponse::*LETHAL* 2)
(intern "*UNKNOWN*" (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(shadow '*UNKNOWN* (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(defconstant mbf_msgs::CheckPathResponse::*UNKNOWN* 3)
(intern "*OUTSIDE*" (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(shadow '*OUTSIDE* (find-package "MBF_MSGS::CHECKPATHRESPONSE"))
(defconstant mbf_msgs::CheckPathResponse::*OUTSIDE* 4)

(defun mbf_msgs::CheckPathResponse-to-symbol (const)
  (cond
        ((= const 0) 'mbf_msgs::CheckPathResponse::*FREE*)
        ((= const 1) 'mbf_msgs::CheckPathResponse::*INSCRIBED*)
        ((= const 2) 'mbf_msgs::CheckPathResponse::*LETHAL*)
        ((= const 3) 'mbf_msgs::CheckPathResponse::*UNKNOWN*)
        ((= const 4) 'mbf_msgs::CheckPathResponse::*OUTSIDE*)
        (t nil)))

(defclass mbf_msgs::CheckPathResponse
  :super ros::object
  :slots (_last_checked _state _cost ))

(defmethod mbf_msgs::CheckPathResponse
  (:init
   (&key
    ((:last_checked __last_checked) 0)
    ((:state __state) 0)
    ((:cost __cost) 0)
    )
   (send-super :init)
   (setq _last_checked (round __last_checked))
   (setq _state (round __state))
   (setq _cost (round __cost))
   self)
  (:last_checked
   (&optional __last_checked)
   (if __last_checked (setq _last_checked __last_checked)) _last_checked)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:serialization-length
   ()
   (+
    ;; uint32 _last_checked
    4
    ;; uint8 _state
    1
    ;; uint32 _cost
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _last_checked
       (write-long _last_checked s)
     ;; uint8 _state
       (write-byte _state s)
     ;; uint32 _cost
       (write-long _cost s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _last_checked
     (setq _last_checked (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _cost
     (setq _cost (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mbf_msgs::CheckPath
  :super ros::object
  :slots ())

(setf (get mbf_msgs::CheckPath :md5sum-) "59ef27f5ce7e3cc3c7f7a0f823bca55c")
(setf (get mbf_msgs::CheckPath :datatype-) "mbf_msgs/CheckPath")
(setf (get mbf_msgs::CheckPath :request) mbf_msgs::CheckPathRequest)
(setf (get mbf_msgs::CheckPath :response) mbf_msgs::CheckPathResponse)

(defmethod mbf_msgs::CheckPathRequest
  (:response () (instance mbf_msgs::CheckPathResponse :init)))

(setf (get mbf_msgs::CheckPathRequest :md5sum-) "59ef27f5ce7e3cc3c7f7a0f823bca55c")
(setf (get mbf_msgs::CheckPathRequest :datatype-) "mbf_msgs/CheckPathRequest")
(setf (get mbf_msgs::CheckPathRequest :definition-)
      "uint8                      LOCAL_COSTMAP  = 1
uint8                      GLOBAL_COSTMAP = 2

nav_msgs/Path              path              # the path to be checked after transforming to costmap frame
float32                    safety_dist       # minimum distance allowed to the closest obstacle (footprint padding)
float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)
float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)
float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)
uint8                      costmap           # costmap in which to check the pose
uint8                      return_on         # abort check on finding a pose with this state or worse (zero is ignored)
uint8                      skip_poses        # skip this number of poses between checks, to speedup processing
bool                       path_cells_only   # check only cells directly traversed by the path, ignoring robot footprint
                                             # (if true, safety_dist is ignored)

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
uint8                      FREE      =  0    # path is completely in traversable space
uint8                      INSCRIBED =  1    # path is partially in inscribed space
uint8                      LETHAL    =  2    # path is partially in collision
uint8                      UNKNOWN   =  3    # path is partially in unknown space
uint8                      OUTSIDE   =  4    # path is partially outside the map

uint32                     last_checked      # index of the first pose along the path with return_on state or worse
uint8                      state             # path worst state (until last_checked): FREE, INFLATED, LETHAL, UNKNOWN...
uint32                     cost              # cost of all cells traversed by path within footprint padded by safety_dist
                                             # or just by the path if path_cells_only is true

")

(setf (get mbf_msgs::CheckPathResponse :md5sum-) "59ef27f5ce7e3cc3c7f7a0f823bca55c")
(setf (get mbf_msgs::CheckPathResponse :datatype-) "mbf_msgs/CheckPathResponse")
(setf (get mbf_msgs::CheckPathResponse :definition-)
      "uint8                      LOCAL_COSTMAP  = 1
uint8                      GLOBAL_COSTMAP = 2

nav_msgs/Path              path              # the path to be checked after transforming to costmap frame
float32                    safety_dist       # minimum distance allowed to the closest obstacle (footprint padding)
float32                    lethal_cost_mult  # cost multiplier for cells marked as lethal obstacle (zero is ignored)
float32                    inscrib_cost_mult # cost multiplier for cells marked as inscribed obstacle (zero is ignored)
float32                    unknown_cost_mult # cost multiplier for cells marked as unknown space (zero is ignored)
uint8                      costmap           # costmap in which to check the pose
uint8                      return_on         # abort check on finding a pose with this state or worse (zero is ignored)
uint8                      skip_poses        # skip this number of poses between checks, to speedup processing
bool                       path_cells_only   # check only cells directly traversed by the path, ignoring robot footprint
                                             # (if true, safety_dist is ignored)

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
uint8                      FREE      =  0    # path is completely in traversable space
uint8                      INSCRIBED =  1    # path is partially in inscribed space
uint8                      LETHAL    =  2    # path is partially in collision
uint8                      UNKNOWN   =  3    # path is partially in unknown space
uint8                      OUTSIDE   =  4    # path is partially outside the map

uint32                     last_checked      # index of the first pose along the path with return_on state or worse
uint8                      state             # path worst state (until last_checked): FREE, INFLATED, LETHAL, UNKNOWN...
uint32                     cost              # cost of all cells traversed by path within footprint padded by safety_dist
                                             # or just by the path if path_cells_only is true

")



(provide :mbf_msgs/CheckPath "59ef27f5ce7e3cc3c7f7a0f823bca55c")


