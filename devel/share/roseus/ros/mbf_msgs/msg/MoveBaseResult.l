;; Auto-generated. Do not edit!


(when (boundp 'mbf_msgs::MoveBaseResult)
  (if (not (find-package "MBF_MSGS"))
    (make-package "MBF_MSGS"))
  (shadow 'MoveBaseResult (find-package "MBF_MSGS")))
(unless (find-package "MBF_MSGS::MOVEBASERESULT")
  (make-package "MBF_MSGS::MOVEBASERESULT"))

(in-package "ROS")
;;//! \htmlinclude MoveBaseResult.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*SUCCESS*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*SUCCESS* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*SUCCESS* 0)
(intern "*FAILURE*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*FAILURE* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*FAILURE* 10)
(intern "*CANCELED*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*CANCELED* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*CANCELED* 11)
(intern "*COLLISION*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*COLLISION* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*COLLISION* 12)
(intern "*OSCILLATION*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*OSCILLATION* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*OSCILLATION* 13)
(intern "*START_BLOCKED*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*START_BLOCKED* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*START_BLOCKED* 14)
(intern "*GOAL_BLOCKED*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*GOAL_BLOCKED* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*GOAL_BLOCKED* 15)
(intern "*TF_ERROR*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*TF_ERROR* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*TF_ERROR* 16)
(intern "*INTERNAL_ERROR*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*INTERNAL_ERROR* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*INTERNAL_ERROR* 17)
(intern "*PLAN_FAILURE*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*PLAN_FAILURE* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*PLAN_FAILURE* 50)
(intern "*CTRL_FAILURE*" (find-package "MBF_MSGS::MOVEBASERESULT"))
(shadow '*CTRL_FAILURE* (find-package "MBF_MSGS::MOVEBASERESULT"))
(defconstant mbf_msgs::MoveBaseResult::*CTRL_FAILURE* 100)

(defun mbf_msgs::MoveBaseResult-to-symbol (const)
  (cond
        ((= const 0) 'mbf_msgs::MoveBaseResult::*SUCCESS*)
        ((= const 10) 'mbf_msgs::MoveBaseResult::*FAILURE*)
        ((= const 11) 'mbf_msgs::MoveBaseResult::*CANCELED*)
        ((= const 12) 'mbf_msgs::MoveBaseResult::*COLLISION*)
        ((= const 13) 'mbf_msgs::MoveBaseResult::*OSCILLATION*)
        ((= const 14) 'mbf_msgs::MoveBaseResult::*START_BLOCKED*)
        ((= const 15) 'mbf_msgs::MoveBaseResult::*GOAL_BLOCKED*)
        ((= const 16) 'mbf_msgs::MoveBaseResult::*TF_ERROR*)
        ((= const 17) 'mbf_msgs::MoveBaseResult::*INTERNAL_ERROR*)
        ((= const 50) 'mbf_msgs::MoveBaseResult::*PLAN_FAILURE*)
        ((= const 100) 'mbf_msgs::MoveBaseResult::*CTRL_FAILURE*)
        (t nil)))

(defclass mbf_msgs::MoveBaseResult
  :super ros::object
  :slots (_outcome _message _dist_to_goal _angle_to_goal _final_pose ))

(defmethod mbf_msgs::MoveBaseResult
  (:init
   (&key
    ((:outcome __outcome) 0)
    ((:message __message) "")
    ((:dist_to_goal __dist_to_goal) 0.0)
    ((:angle_to_goal __angle_to_goal) 0.0)
    ((:final_pose __final_pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _outcome (round __outcome))
   (setq _message (string __message))
   (setq _dist_to_goal (float __dist_to_goal))
   (setq _angle_to_goal (float __angle_to_goal))
   (setq _final_pose __final_pose)
   self)
  (:outcome
   (&optional __outcome)
   (if __outcome (setq _outcome __outcome)) _outcome)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:dist_to_goal
   (&optional __dist_to_goal)
   (if __dist_to_goal (setq _dist_to_goal __dist_to_goal)) _dist_to_goal)
  (:angle_to_goal
   (&optional __angle_to_goal)
   (if __angle_to_goal (setq _angle_to_goal __angle_to_goal)) _angle_to_goal)
  (:final_pose
   (&rest __final_pose)
   (if (keywordp (car __final_pose))
       (send* _final_pose __final_pose)
     (progn
       (if __final_pose (setq _final_pose (car __final_pose)))
       _final_pose)))
  (:serialization-length
   ()
   (+
    ;; uint32 _outcome
    4
    ;; string _message
    4 (length _message)
    ;; float32 _dist_to_goal
    4
    ;; float32 _angle_to_goal
    4
    ;; geometry_msgs/PoseStamped _final_pose
    (send _final_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _outcome
       (write-long _outcome s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float32 _dist_to_goal
       (sys::poke _dist_to_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_to_goal
       (sys::poke _angle_to_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/PoseStamped _final_pose
       (send _final_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _outcome
     (setq _outcome (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _dist_to_goal
     (setq _dist_to_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_to_goal
     (setq _angle_to_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/PoseStamped _final_pose
     (send _final_pose :deserialize buf ptr-) (incf ptr- (send _final_pose :serialization-length))
   ;;
   self)
  )

(setf (get mbf_msgs::MoveBaseResult :md5sum-) "c65d301ffa20590244253c6a99c37c5e")
(setf (get mbf_msgs::MoveBaseResult :datatype-) "mbf_msgs/MoveBaseResult")
(setf (get mbf_msgs::MoveBaseResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Predefined success codes:
uint8 SUCCESS        = 0

# Predefined general error codes:
uint8 FAILURE        = 10
uint8 CANCELED       = 11
uint8 COLLISION      = 12
uint8 OSCILLATION    = 13
uint8 START_BLOCKED  = 14
uint8 GOAL_BLOCKED   = 15
uint8 TF_ERROR       = 16
uint8 INTERNAL_ERROR = 17
# 21..49 are reserved for future general error codes

# Planning/controlling failures:
uint8 PLAN_FAILURE   = 50
# 51..99 are reserved as planner specific errors

uint8 CTRL_FAILURE   = 100
# 101..149 are reserved as controller specific errors

uint32 outcome
string message

# Configuration upon action completion
float32 dist_to_goal
float32 angle_to_goal
geometry_msgs/PoseStamped final_pose


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



(provide :mbf_msgs/MoveBaseResult "c65d301ffa20590244253c6a99c37c5e")


