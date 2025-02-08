;; Auto-generated. Do not edit!


(when (boundp 'mbf_msgs::GetPathResult)
  (if (not (find-package "MBF_MSGS"))
    (make-package "MBF_MSGS"))
  (shadow 'GetPathResult (find-package "MBF_MSGS")))
(unless (find-package "MBF_MSGS::GETPATHRESULT")
  (make-package "MBF_MSGS::GETPATHRESULT"))

(in-package "ROS")
;;//! \htmlinclude GetPathResult.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(intern "*SUCCESS*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*SUCCESS* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*SUCCESS* 0)
(intern "*FAILURE*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*FAILURE* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*FAILURE* 50)
(intern "*CANCELED*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*CANCELED* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*CANCELED* 51)
(intern "*INVALID_START*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*INVALID_START* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*INVALID_START* 52)
(intern "*INVALID_GOAL*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*INVALID_GOAL* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*INVALID_GOAL* 53)
(intern "*NO_PATH_FOUND*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*NO_PATH_FOUND* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*NO_PATH_FOUND* 54)
(intern "*PAT_EXCEEDED*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*PAT_EXCEEDED* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*PAT_EXCEEDED* 55)
(intern "*EMPTY_PATH*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*EMPTY_PATH* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*EMPTY_PATH* 56)
(intern "*TF_ERROR*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*TF_ERROR* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*TF_ERROR* 57)
(intern "*NOT_INITIALIZED*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*NOT_INITIALIZED* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*NOT_INITIALIZED* 58)
(intern "*INVALID_PLUGIN*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*INVALID_PLUGIN* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*INVALID_PLUGIN* 59)
(intern "*INTERNAL_ERROR*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*INTERNAL_ERROR* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*INTERNAL_ERROR* 60)
(intern "*OUT_OF_MAP*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*OUT_OF_MAP* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*OUT_OF_MAP* 61)
(intern "*MAP_ERROR*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*MAP_ERROR* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*MAP_ERROR* 62)
(intern "*STOPPED*" (find-package "MBF_MSGS::GETPATHRESULT"))
(shadow '*STOPPED* (find-package "MBF_MSGS::GETPATHRESULT"))
(defconstant mbf_msgs::GetPathResult::*STOPPED* 63)

(defun mbf_msgs::GetPathResult-to-symbol (const)
  (cond
        ((= const 0) 'mbf_msgs::GetPathResult::*SUCCESS*)
        ((= const 50) 'mbf_msgs::GetPathResult::*FAILURE*)
        ((= const 51) 'mbf_msgs::GetPathResult::*CANCELED*)
        ((= const 52) 'mbf_msgs::GetPathResult::*INVALID_START*)
        ((= const 53) 'mbf_msgs::GetPathResult::*INVALID_GOAL*)
        ((= const 54) 'mbf_msgs::GetPathResult::*NO_PATH_FOUND*)
        ((= const 55) 'mbf_msgs::GetPathResult::*PAT_EXCEEDED*)
        ((= const 56) 'mbf_msgs::GetPathResult::*EMPTY_PATH*)
        ((= const 57) 'mbf_msgs::GetPathResult::*TF_ERROR*)
        ((= const 58) 'mbf_msgs::GetPathResult::*NOT_INITIALIZED*)
        ((= const 59) 'mbf_msgs::GetPathResult::*INVALID_PLUGIN*)
        ((= const 60) 'mbf_msgs::GetPathResult::*INTERNAL_ERROR*)
        ((= const 61) 'mbf_msgs::GetPathResult::*OUT_OF_MAP*)
        ((= const 62) 'mbf_msgs::GetPathResult::*MAP_ERROR*)
        ((= const 63) 'mbf_msgs::GetPathResult::*STOPPED*)
        (t nil)))

(defclass mbf_msgs::GetPathResult
  :super ros::object
  :slots (_outcome _message _path _cost ))

(defmethod mbf_msgs::GetPathResult
  (:init
   (&key
    ((:outcome __outcome) 0)
    ((:message __message) "")
    ((:path __path) (instance nav_msgs::Path :init))
    ((:cost __cost) 0.0)
    )
   (send-super :init)
   (setq _outcome (round __outcome))
   (setq _message (string __message))
   (setq _path __path)
   (setq _cost (float __cost))
   self)
  (:outcome
   (&optional __outcome)
   (if __outcome (setq _outcome __outcome)) _outcome)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:serialization-length
   ()
   (+
    ;; uint32 _outcome
    4
    ;; string _message
    4 (length _message)
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ;; float64 _cost
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _outcome
       (write-long _outcome s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;; float64 _cost
       (sys::poke _cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _outcome
     (setq _outcome (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;; float64 _cost
     (setq _cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mbf_msgs::GetPathResult :md5sum-) "b737d51aec954f878a4cd57e83f5767c")
(setf (get mbf_msgs::GetPathResult :datatype-) "mbf_msgs/GetPathResult")
(setf (get mbf_msgs::GetPathResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Predefined success codes:
uint8 SUCCESS         = 0
# 1..9 are reserved as plugin specific non-error results

# Possible error codes:
uint8 FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
uint8 CANCELED        = 51  # The action has been canceled by a action client
uint8 INVALID_START   = 52  #
uint8 INVALID_GOAL    = 53
uint8 NO_PATH_FOUND   = 54
uint8 PAT_EXCEEDED    = 55
uint8 EMPTY_PATH      = 56
uint8 TF_ERROR        = 57
uint8 NOT_INITIALIZED = 58
uint8 INVALID_PLUGIN  = 59
uint8 INTERNAL_ERROR  = 60
uint8 OUT_OF_MAP      = 61
uint8 MAP_ERROR       = 62
uint8 STOPPED         = 63  # The planner execution has been stopped rigorously.

# 71..99 are reserved as plugin specific errors

uint32 outcome
string message

nav_msgs/Path path

float64 cost


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

")



(provide :mbf_msgs/GetPathResult "b737d51aec954f878a4cd57e83f5767c")


