# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dec_control:msg/NetworkTopology.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'adjacency_matrix'
# Member 'link_quality'
# Member 'link_latency'
# Member 'link_reliability'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NetworkTopology(type):
    """Metaclass of message 'NetworkTopology'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dec_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dec_control.msg.NetworkTopology')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__network_topology
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__network_topology
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__network_topology
            cls._TYPE_SUPPORT = module.type_support_msg__msg__network_topology
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__network_topology

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NetworkTopology(metaclass=Metaclass_NetworkTopology):
    """Message class 'NetworkTopology'."""

    __slots__ = [
        '_num_robots',
        '_adjacency_matrix',
        '_link_quality',
        '_link_latency',
        '_link_reliability',
        '_timestamp',
    ]

    _fields_and_field_types = {
        'num_robots': 'int32',
        'adjacency_matrix': 'sequence<float>',
        'link_quality': 'sequence<float>',
        'link_latency': 'sequence<float>',
        'link_reliability': 'sequence<float>',
        'timestamp': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.num_robots = kwargs.get('num_robots', int())
        self.adjacency_matrix = array.array('f', kwargs.get('adjacency_matrix', []))
        self.link_quality = array.array('f', kwargs.get('link_quality', []))
        self.link_latency = array.array('f', kwargs.get('link_latency', []))
        self.link_reliability = array.array('f', kwargs.get('link_reliability', []))
        self.timestamp = kwargs.get('timestamp', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.num_robots != other.num_robots:
            return False
        if self.adjacency_matrix != other.adjacency_matrix:
            return False
        if self.link_quality != other.link_quality:
            return False
        if self.link_latency != other.link_latency:
            return False
        if self.link_reliability != other.link_reliability:
            return False
        if self.timestamp != other.timestamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def num_robots(self):
        """Message field 'num_robots'."""
        return self._num_robots

    @num_robots.setter
    def num_robots(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_robots' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_robots' field must be an integer in [-2147483648, 2147483647]"
        self._num_robots = value

    @builtins.property
    def adjacency_matrix(self):
        """Message field 'adjacency_matrix'."""
        return self._adjacency_matrix

    @adjacency_matrix.setter
    def adjacency_matrix(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'adjacency_matrix' array.array() must have the type code of 'f'"
            self._adjacency_matrix = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'adjacency_matrix' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._adjacency_matrix = array.array('f', value)

    @builtins.property
    def link_quality(self):
        """Message field 'link_quality'."""
        return self._link_quality

    @link_quality.setter
    def link_quality(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'link_quality' array.array() must have the type code of 'f'"
            self._link_quality = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'link_quality' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._link_quality = array.array('f', value)

    @builtins.property
    def link_latency(self):
        """Message field 'link_latency'."""
        return self._link_latency

    @link_latency.setter
    def link_latency(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'link_latency' array.array() must have the type code of 'f'"
            self._link_latency = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'link_latency' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._link_latency = array.array('f', value)

    @builtins.property
    def link_reliability(self):
        """Message field 'link_reliability'."""
        return self._link_reliability

    @link_reliability.setter
    def link_reliability(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'link_reliability' array.array() must have the type code of 'f'"
            self._link_reliability = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'link_reliability' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._link_reliability = array.array('f', value)

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'timestamp' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._timestamp = value
