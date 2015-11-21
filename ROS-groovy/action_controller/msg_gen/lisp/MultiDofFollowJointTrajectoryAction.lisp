; Auto-generated. Do not edit!


(cl:in-package action_controller-msg)


;//! \htmlinclude MultiDofFollowJointTrajectoryAction.msg.html

(cl:defclass <MultiDofFollowJointTrajectoryAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type action_controller-msg:MultiDofFollowJointTrajectoryActionGoal
    :initform (cl:make-instance 'action_controller-msg:MultiDofFollowJointTrajectoryActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type action_controller-msg:MultiDofFollowJointTrajectoryActionResult
    :initform (cl:make-instance 'action_controller-msg:MultiDofFollowJointTrajectoryActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type action_controller-msg:MultiDofFollowJointTrajectoryActionFeedback
    :initform (cl:make-instance 'action_controller-msg:MultiDofFollowJointTrajectoryActionFeedback)))
)

(cl:defclass MultiDofFollowJointTrajectoryAction (<MultiDofFollowJointTrajectoryAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiDofFollowJointTrajectoryAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiDofFollowJointTrajectoryAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_controller-msg:<MultiDofFollowJointTrajectoryAction> is deprecated: use action_controller-msg:MultiDofFollowJointTrajectoryAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <MultiDofFollowJointTrajectoryAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_controller-msg:action_goal-val is deprecated.  Use action_controller-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <MultiDofFollowJointTrajectoryAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_controller-msg:action_result-val is deprecated.  Use action_controller-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <MultiDofFollowJointTrajectoryAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_controller-msg:action_feedback-val is deprecated.  Use action_controller-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiDofFollowJointTrajectoryAction>) ostream)
  "Serializes a message object of type '<MultiDofFollowJointTrajectoryAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiDofFollowJointTrajectoryAction>) istream)
  "Deserializes a message object of type '<MultiDofFollowJointTrajectoryAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiDofFollowJointTrajectoryAction>)))
  "Returns string type for a message object of type '<MultiDofFollowJointTrajectoryAction>"
  "action_controller/MultiDofFollowJointTrajectoryAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiDofFollowJointTrajectoryAction)))
  "Returns string type for a message object of type 'MultiDofFollowJointTrajectoryAction"
  "action_controller/MultiDofFollowJointTrajectoryAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiDofFollowJointTrajectoryAction>)))
  "Returns md5sum for a message object of type '<MultiDofFollowJointTrajectoryAction>"
  "b718ef3648f19dfc8236babeb519eea6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiDofFollowJointTrajectoryAction)))
  "Returns md5sum for a message object of type 'MultiDofFollowJointTrajectoryAction"
  "b718ef3648f19dfc8236babeb519eea6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiDofFollowJointTrajectoryAction>)))
  "Returns full string definition for message of type '<MultiDofFollowJointTrajectoryAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%MultiDofFollowJointTrajectoryActionGoal action_goal~%MultiDofFollowJointTrajectoryActionResult action_result~%MultiDofFollowJointTrajectoryActionFeedback action_feedback~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%MultiDofFollowJointTrajectoryGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# The joint trajectory to follow~%moveit_msgs/MultiDOFJointTrajectory trajectory~%~%================================================================================~%MSG: moveit_msgs/MultiDOFJointTrajectory~%# The header is used to specify the reference time for the trajectory durations~%Header header~%~%#A representation of a multi-dof joint trajectory~%string[] joint_names~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: moveit_msgs/MultiDOFJointTrajectoryPoint~%geometry_msgs/Transform[] transforms~%duration time_from_start~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%MultiDofFollowJointTrajectoryResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int32 error_code~%int32 SUCCESSFUL = 0~%int32 INVALID_GOAL = -1~%int32 INVALID_JOINTS = -2~%int32 OLD_HEADER_TIMESTAMP = -3~%int32 PATH_TOLERANCE_VIOLATED = -4~%int32 GOAL_TOLERANCE_VIOLATED = -5~%~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%MultiDofFollowJointTrajectoryFeedback feedback~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%Header header~%string[] joint_names~%moveit_msgs/MultiDOFJointTrajectoryPoint desired~%moveit_msgs/MultiDOFJointTrajectoryPoint actual~%moveit_msgs/MultiDOFJointTrajectoryPoint error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiDofFollowJointTrajectoryAction)))
  "Returns full string definition for message of type 'MultiDofFollowJointTrajectoryAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%MultiDofFollowJointTrajectoryActionGoal action_goal~%MultiDofFollowJointTrajectoryActionResult action_result~%MultiDofFollowJointTrajectoryActionFeedback action_feedback~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%MultiDofFollowJointTrajectoryGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# The joint trajectory to follow~%moveit_msgs/MultiDOFJointTrajectory trajectory~%~%================================================================================~%MSG: moveit_msgs/MultiDOFJointTrajectory~%# The header is used to specify the reference time for the trajectory durations~%Header header~%~%#A representation of a multi-dof joint trajectory~%string[] joint_names~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: moveit_msgs/MultiDOFJointTrajectoryPoint~%geometry_msgs/Transform[] transforms~%duration time_from_start~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%MultiDofFollowJointTrajectoryResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int32 error_code~%int32 SUCCESSFUL = 0~%int32 INVALID_GOAL = -1~%int32 INVALID_JOINTS = -2~%int32 OLD_HEADER_TIMESTAMP = -3~%int32 PATH_TOLERANCE_VIOLATED = -4~%int32 GOAL_TOLERANCE_VIOLATED = -5~%~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%MultiDofFollowJointTrajectoryFeedback feedback~%~%================================================================================~%MSG: action_controller/MultiDofFollowJointTrajectoryFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%Header header~%string[] joint_names~%moveit_msgs/MultiDOFJointTrajectoryPoint desired~%moveit_msgs/MultiDOFJointTrajectoryPoint actual~%moveit_msgs/MultiDOFJointTrajectoryPoint error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiDofFollowJointTrajectoryAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiDofFollowJointTrajectoryAction>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiDofFollowJointTrajectoryAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
