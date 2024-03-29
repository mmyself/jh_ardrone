"""autogenerated by genmsg_py from Tags.msg. Do not edit."""
import roslib.message
import struct

import ar_recog.msg
import std_msgs.msg

class Tags(roslib.message.Message):
  _md5sum = "c72ebed973449ed4da4f4054bec6620a"
  _type = "ar_recog/Tags"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
uint32 image_width
uint32 image_height
float64 angle_of_view
uint32 tag_count
Tag[] tags

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
MSG: ar_recog/Tag
uint32 id
float64 cf
uint32 x
uint32 y
uint32 diameter
uint32 distance
float64 xRot 
float64 yRot 
float64 zRot 
float64[8] cwCorners

"""
  __slots__ = ['header','image_width','image_height','angle_of_view','tag_count','tags']
  _slot_types = ['Header','uint32','uint32','float64','uint32','ar_recog/Tag[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,image_width,image_height,angle_of_view,tag_count,tags
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Tags, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.image_width is None:
        self.image_width = 0
      if self.image_height is None:
        self.image_height = 0
      if self.angle_of_view is None:
        self.angle_of_view = 0.
      if self.tag_count is None:
        self.tag_count = 0
      if self.tags is None:
        self.tags = []
    else:
      self.header = std_msgs.msg._Header.Header()
      self.image_width = 0
      self.image_height = 0
      self.angle_of_view = 0.
      self.tag_count = 0
      self.tags = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2IdI.pack(_x.image_width, _x.image_height, _x.angle_of_view, _x.tag_count))
      length = len(self.tags)
      buff.write(_struct_I.pack(length))
      for val1 in self.tags:
        _x = val1
        buff.write(_struct_Id4I3d.pack(_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot))
        buff.write(_struct_8d.pack(*val1.cwCorners))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
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
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 20
      (_x.image_width, _x.image_height, _x.angle_of_view, _x.tag_count,) = _struct_2IdI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.tags = []
      for i in xrange(0, length):
        val1 = ar_recog.msg.Tag()
        _x = val1
        start = end
        end += 52
        (_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot,) = _struct_Id4I3d.unpack(str[start:end])
        start = end
        end += 64
        val1.cwCorners = _struct_8d.unpack(str[start:end])
        self.tags.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2IdI.pack(_x.image_width, _x.image_height, _x.angle_of_view, _x.tag_count))
      length = len(self.tags)
      buff.write(_struct_I.pack(length))
      for val1 in self.tags:
        _x = val1
        buff.write(_struct_Id4I3d.pack(_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot))
        buff.write(val1.cwCorners.tostring())
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
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
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 20
      (_x.image_width, _x.image_height, _x.angle_of_view, _x.tag_count,) = _struct_2IdI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.tags = []
      for i in xrange(0, length):
        val1 = ar_recog.msg.Tag()
        _x = val1
        start = end
        end += 52
        (_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot,) = _struct_Id4I3d.unpack(str[start:end])
        start = end
        end += 64
        val1.cwCorners = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=8)
        self.tags.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_8d = struct.Struct("<8d")
_struct_3I = struct.Struct("<3I")
_struct_Id4I3d = struct.Struct("<Id4I3d")
_struct_2IdI = struct.Struct("<2IdI")
