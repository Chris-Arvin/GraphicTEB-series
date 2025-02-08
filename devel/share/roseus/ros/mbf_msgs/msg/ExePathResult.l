;; Auto-generated. Do not edit!


(when (boundp 'mbf_msgs::ExePathResult)
  (if (not (find-package "MBF_MSGS"))
    (make-package "MBF_MSGS"))
  (shadow 'ExePathResult (find-package "MBF_MSGS")))
(unless (find-package "MBF_MSGS::EXEPATHRESULT")
  (make-package "MBF_MSGS::EXEPATHRESULT"))

(in-package "ROS")
;;//! \htmlinclude ExePathResult.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*SUCCESS*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*SUCCESS* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*SUCCESS* 0)
(intern "*FAILURE*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*FAILURE* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*FAILURE* 100)
(intern "*CANCELED*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*CANCELED* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*CANCELED* 101)
(intern "*NO_VALID_CMD*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*NO_VALID_CMD* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*NO_VALID_CMD* 102)
(intern "*PAT_EXCEEDED*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*PAT_EXCEEDED* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*PAT_EXCEEDED* 103)
(intern "*COLLISION*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*COLLISION* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*COLLISION* 104)
(intern "*OSCILLATION*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*OSCILLATION* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*OSCILLATION* 105)
(intern "*ROBOT_STUCK*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*ROBOT_STUCK* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*ROBOT_STUCK* 106)
(intern "*MISSED_GOAL*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*MISSED_GOAL* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*MISSED_GOAL* 107)
(intern "*MISSED_PATH*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*MISSED_PATH* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*MISSED_PATH* 108)
(intern "*BLOCKED_PATH*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*BLOCKED_PATH* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*BLOCKED_PATH* 109)
(intern "*INVALID_PATH*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*INVALID_PATH* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*INVALID_PATH* 110)
(intern "*TF_ERROR*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*TF_ERROR* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*TF_ERROR* 111)
(intern "*NOT_INITIALIZED*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*NOT_INITIALIZED* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*NOT_INITIALIZED* 112)
(intern "*INVALID_PLUGIN*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*INVALID_PLUGIN* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*INVALID_PLUGIN* 113)
(intern "*INTERNAL_ERROR*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*INTERNAL_ERROR* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*INTERNAL_ERROR* 114)
(intern "*OUT_OF_MAP*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*OUT_OF_MAP* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*OUT_OF_MAP* 115)
(intern "*MAP_ERROR*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*MAP_ERROR* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*MAP_ERROR* 116)
(intern "*STOPPED*" (find-package "MBF_MSGS::EXEPATHRESULT"))
(shadow '*STOPPED* (find-package "MBF_MSGS::EXEPATHRESULT"))
(defconstant mbf_msgs::ExePathResult::*STOPPED* 117)

(defun mbf_msgs::ExePathResult-to-symbol (const)
  (cond
        ((= const 0) 'mbf_msgs::ExePathResult::*SUCCESS*)
        ((= const 100) 'mbf_msgs::ExePathResult::*FAILURE*)
        ((= const 101) 'mbf_msgs::ExePathResult::*CANCELED*)
        ((= const 102) 'mbf_msgs::ExePathResult::*NO_VALID_CMD*)
        ((= const 103) 'mbf_msgs::ExePathResult::*PAT_EXCEEDED*)
        ((= const 104) 'mbf_msgs::ExePathResult::*COLLISION*)
        ((= const 105) 'mbf_msgs::ExePathResult::*OSCILLATION*)
        ((= const 106) 'mbf_msgs::ExePathResult::*ROBOT_STUCK*)
        ((= const 107) 'mbf_msgs::ExePathResult::*MISSED_GOAL*)
        ((= const 108) 'mbf_msgs::ExePathResult::*MISSED_PATH*)
        ((= const 109) 'mbf_msgs::ExePathResult::*BLOCKED_PATH*)
        ((= const 110) 'mbf_msgs::ExePathResult::*INVALID_PATH*)
        ((= const 111) 'mbf_msgs::ExePathResult::*TF_ERROR*)
        ((= const 112) 'mbf_msgs::ExePathResult::*NOT_INITIALIZED*)
        ((= const 113) 'mbf_msgs::ExePathResult::*INVALID_PLUGIN*)
        ((= const 114) 'mbf_msgs::ExePathResult::*INTERNAL_ERROR*)
        ((= const 115) 'mbf_msgs::ExePathResult::*OUT_OF_MAP*)
        ((= const 116) 'mbf_msgs::ExePathResult::*MAP_ERROR*)
        ((= const 117) 'mbf_msgs::ExePathResult::*STOPPED*)
        (t nil)))

(defclass mbf_msgs::ExePathResult
  :super ros::object
  :slots (_outcome _message _final_pose _dist_to_goal _angle_to_goal ))

(defmethod mbf_msgs::ExePathResult
  (:init
   (&key
    ((:outcome __outcome) 0)
    ((:message __message) "")
    ((:final_pose __final_pose) (instance geometry_msgs::PoseStamped :init))
    ((:dist_to_goal __dist_to_goal) 0.0)
    ((:angle_to_goal __angle_to_goal) 0.0)
    )
   (send-super :init)
   (setq _outcome (round __outcome))
   (setq _message (string __message))
   (setq _final_pose __final_pose)
   (setq _dist_to_goal (float __dist_to_goal))
   (setq _angle_to_goal (float __angle_to_goal))
   self)
  (:outcome
   (&optional __outcome)
   (if __outcome (setq _outcome __outcome)) _outcome)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:final_pose
   (&rest __final_pose)
   (if (keywordp (car __final_pose))
       (send* _final_pose __final_pose)
     (progn
       (if __final_pose (setq _final_pose (car __final_pose)))
       _final_pose)))
  (:dist_to_goal
   (&optional __dist_to_goal)
   (if __dist_to_goal (setq _dist_to_goal __dist_to_goal)) _dist_to_goal)
  (:angle_to_goal
   (&optional __angle_to_goal)
   (if __angle_to_goal (setq _angle_to_goal __angle_to_goal)) _angle_to_goal)
  (:serialization-length
   ()
   (+
    ;; uint32 _outcome
    4
    ;; string _message
    4 (length _message)
    ;; geometry_msgs/PoseStamped _final_pose
    (send _final_pose :serialization-length)
    ;; float32 _dist_to_goal
    4
    ;; float32 _angle_to_goal
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _outcome
       (write-long _outcome s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; geometry_msgs/PoseStamped _final_pose
       (send _final_pose :serialize s)
     ;; float32 _dist_to_goal
       (sys::poke _dist_to_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_to_goal
       (sys::poke _angle_to_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _outcome
     (setq _outcome (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/PoseStamped _final_pose
     (send _final_pose :deserialize buf ptr-) (incf ptr- (send _final_pose :serialization-length))
   ;; float32 _dist_to_goal
     (setq _dist_to_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_to_goal
     (setq _angle_to_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mbf_msgs::ExePathResult :md5sum-) "b22f308686bb4e3a7364ea944ef68fd0")
(setf (get mbf_msgs::ExePathResult :datatype-) "mbf_msgs/ExePathResult")
(setf (get mbf_msgs::ExePathResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Predefined success codes:
uint8 SUCCESS         = 0
# 1..9 are reserved as plugin specific non-error results

# Predefined error codes:
uint8 FAILURE         = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
uint8 CANCELED        = 101
uint8 NO_VALID_CMD    = 102
uint8 PAT_EXCEEDED    = 103
uint8 COLLISION       = 104
uint8 OSCILLATION     = 105
uint8 ROBOT_STUCK     = 106
uint8 MISSED_GOAL     = 107
uint8 MISSED_PATH     = 108
uint8 BLOCKED_PATH    = 109
uint8 INVALID_PATH    = 110
uint8 TF_ERROR        = 111
uint8 NOT_INITIALIZED = 112
uint8 INVALID_PLUGIN  = 113
uint8 INTERNAL_ERROR  = 114
uint8 OUT_OF_MAP      = 115  # The start and / or the goal are outside the map
uint8 MAP_ERROR       = 116  # The map is not running properly
uint8 STOPPED         = 117  # The controller execution has been stopped rigorously.

# 121..149 are reserved as plugin specific errors

uint32 outcome
string message

geometry_msgs/PoseStamped  final_pose
float32 dist_to_goal
float32 angle_to_goal


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

")



(provide :mbf_msgs/ExePathResult "b22f308686bb4e3a7364ea944ef68fd0")


