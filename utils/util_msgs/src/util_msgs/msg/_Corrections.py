"""autogenerated by genpy from util_msgs/Corrections.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Corrections(genpy.Message):
  _md5sum = "61e86887a75fe520847d3256306360f5"
  _type = "util_msgs/Corrections"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 kf_correction
float64[2] angle_corrections

"""
  __slots__ = ['kf_correction','angle_corrections']
  _slot_types = ['float64','float64[2]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       kf_correction,angle_corrections

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Corrections, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.kf_correction is None:
        self.kf_correction = 0.
      if self.angle_corrections is None:
        self.angle_corrections = [0.,0.]
    else:
      self.kf_correction = 0.
      self.angle_corrections = [0.,0.]

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
      buff.write(_struct_d.pack(self.kf_correction))
      buff.write(_struct_2d.pack(*self.angle_corrections))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 8
      (self.kf_correction,) = _struct_d.unpack(str[start:end])
      start = end
      end += 16
      self.angle_corrections = _struct_2d.unpack(str[start:end])
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
      buff.write(_struct_d.pack(self.kf_correction))
      buff.write(self.angle_corrections.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 8
      (self.kf_correction,) = _struct_d.unpack(str[start:end])
      start = end
      end += 16
      self.angle_corrections = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=2)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
_struct_d = struct.Struct("<d")
