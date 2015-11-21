"""autogenerated by genpy from action_controller/MultiDofFollowJointTrajectoryActionFeedback.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import actionlib_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import std_msgs.msg
import genpy
import action_controller.msg

class MultiDofFollowJointTrajectoryActionFeedback(genpy.Message):
  _md5sum = "bb90301ebf9ac17d9c3bcafa7a141cba"
  _type = "action_controller/MultiDofFollowJointTrajectoryActionFeedback"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
MultiDofFollowJointTrajectoryFeedback feedback

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text


================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id


================================================================================
MSG: action_controller/MultiDofFollowJointTrajectoryFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
Header header
string[] joint_names
moveit_msgs/MultiDOFJointTrajectoryPoint desired
moveit_msgs/MultiDOFJointTrajectoryPoint actual
moveit_msgs/MultiDOFJointTrajectoryPoint error


================================================================================
MSG: moveit_msgs/MultiDOFJointTrajectoryPoint
geometry_msgs/Transform[] transforms
duration time_from_start

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

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

"""
  __slots__ = ['header','status','feedback']
  _slot_types = ['std_msgs/Header','actionlib_msgs/GoalStatus','action_controller/MultiDofFollowJointTrajectoryFeedback']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,status,feedback

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MultiDofFollowJointTrajectoryActionFeedback, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.status is None:
        self.status = actionlib_msgs.msg.GoalStatus()
      if self.feedback is None:
        self.feedback = action_controller.msg.MultiDofFollowJointTrajectoryFeedback()
    else:
      self.header = std_msgs.msg.Header()
      self.status = actionlib_msgs.msg.GoalStatus()
      self.feedback = action_controller.msg.MultiDofFollowJointTrajectoryFeedback()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.status.goal_id.stamp.secs, _x.status.goal_id.stamp.nsecs))
      _x = self.status.goal_id.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.status.status))
      _x = self.status.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.feedback.header.seq, _x.feedback.header.stamp.secs, _x.feedback.header.stamp.nsecs))
      _x = self.feedback.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.feedback.joint_names)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.joint_names:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.feedback.desired.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.desired.transforms:
        _v1 = val1.translation
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v2 = val1.rotation
        _x = _v2
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_2i.pack(_x.feedback.desired.time_from_start.secs, _x.feedback.desired.time_from_start.nsecs))
      length = len(self.feedback.actual.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.actual.transforms:
        _v3 = val1.translation
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v4 = val1.rotation
        _x = _v4
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_2i.pack(_x.feedback.actual.time_from_start.secs, _x.feedback.actual.time_from_start.nsecs))
      length = len(self.feedback.error.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.error.transforms:
        _v5 = val1.translation
        _x = _v5
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v6 = val1.rotation
        _x = _v6
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_2i.pack(_x.feedback.error.time_from_start.secs, _x.feedback.error.time_from_start.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.status is None:
        self.status = actionlib_msgs.msg.GoalStatus()
      if self.feedback is None:
        self.feedback = action_controller.msg.MultiDofFollowJointTrajectoryFeedback()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.status.goal_id.stamp.secs, _x.status.goal_id.stamp.nsecs,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status.goal_id.id = str[start:end].decode('utf-8')
      else:
        self.status.goal_id.id = str[start:end]
      start = end
      end += 1
      (self.status.status,) = _struct_B.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status.text = str[start:end].decode('utf-8')
      else:
        self.status.text = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.feedback.header.seq, _x.feedback.header.stamp.secs, _x.feedback.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.feedback.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.feedback.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.joint_names = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.feedback.joint_names.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.desired.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v7 = val1.translation
        _x = _v7
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v8 = val1.rotation
        _x = _v8
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.feedback.desired.transforms.append(val1)
      _x = self
      start = end
      end += 8
      (_x.feedback.desired.time_from_start.secs, _x.feedback.desired.time_from_start.nsecs,) = _struct_2i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.actual.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v9 = val1.translation
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v10 = val1.rotation
        _x = _v10
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.feedback.actual.transforms.append(val1)
      _x = self
      start = end
      end += 8
      (_x.feedback.actual.time_from_start.secs, _x.feedback.actual.time_from_start.nsecs,) = _struct_2i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.error.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v11 = val1.translation
        _x = _v11
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v12 = val1.rotation
        _x = _v12
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.feedback.error.transforms.append(val1)
      _x = self
      start = end
      end += 8
      (_x.feedback.error.time_from_start.secs, _x.feedback.error.time_from_start.nsecs,) = _struct_2i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.status.goal_id.stamp.secs, _x.status.goal_id.stamp.nsecs))
      _x = self.status.goal_id.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.status.status))
      _x = self.status.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.feedback.header.seq, _x.feedback.header.stamp.secs, _x.feedback.header.stamp.nsecs))
      _x = self.feedback.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.feedback.joint_names)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.joint_names:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.feedback.desired.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.desired.transforms:
        _v13 = val1.translation
        _x = _v13
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v14 = val1.rotation
        _x = _v14
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_2i.pack(_x.feedback.desired.time_from_start.secs, _x.feedback.desired.time_from_start.nsecs))
      length = len(self.feedback.actual.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.actual.transforms:
        _v15 = val1.translation
        _x = _v15
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v16 = val1.rotation
        _x = _v16
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_2i.pack(_x.feedback.actual.time_from_start.secs, _x.feedback.actual.time_from_start.nsecs))
      length = len(self.feedback.error.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.feedback.error.transforms:
        _v17 = val1.translation
        _x = _v17
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v18 = val1.rotation
        _x = _v18
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_2i.pack(_x.feedback.error.time_from_start.secs, _x.feedback.error.time_from_start.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.status is None:
        self.status = actionlib_msgs.msg.GoalStatus()
      if self.feedback is None:
        self.feedback = action_controller.msg.MultiDofFollowJointTrajectoryFeedback()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.status.goal_id.stamp.secs, _x.status.goal_id.stamp.nsecs,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status.goal_id.id = str[start:end].decode('utf-8')
      else:
        self.status.goal_id.id = str[start:end]
      start = end
      end += 1
      (self.status.status,) = _struct_B.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status.text = str[start:end].decode('utf-8')
      else:
        self.status.text = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.feedback.header.seq, _x.feedback.header.stamp.secs, _x.feedback.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.feedback.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.feedback.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.joint_names = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.feedback.joint_names.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.desired.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v19 = val1.translation
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v20 = val1.rotation
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.feedback.desired.transforms.append(val1)
      _x = self
      start = end
      end += 8
      (_x.feedback.desired.time_from_start.secs, _x.feedback.desired.time_from_start.nsecs,) = _struct_2i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.actual.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v21 = val1.translation
        _x = _v21
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v22 = val1.rotation
        _x = _v22
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.feedback.actual.transforms.append(val1)
      _x = self
      start = end
      end += 8
      (_x.feedback.actual.time_from_start.secs, _x.feedback.actual.time_from_start.nsecs,) = _struct_2i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.feedback.error.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v23 = val1.translation
        _x = _v23
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v24 = val1.rotation
        _x = _v24
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.feedback.error.transforms.append(val1)
      _x = self
      start = end
      end += 8
      (_x.feedback.error.time_from_start.secs, _x.feedback.error.time_from_start.nsecs,) = _struct_2i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
_struct_2I = struct.Struct("<2I")
_struct_3I = struct.Struct("<3I")
_struct_4d = struct.Struct("<4d")
_struct_2i = struct.Struct("<2i")
_struct_3d = struct.Struct("<3d")
