# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dec_control:msg/RecoveryStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'degraded_capabilities'
# Member 'infeasible_tasks'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RecoveryStatus(type):
    """Metaclass of message 'RecoveryStatus'."""

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
                'dec_control.msg.RecoveryStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__recovery_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__recovery_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__recovery_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__recovery_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__recovery_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RecoveryStatus(metaclass=Metaclass_RecoveryStatus):
    """Message class 'RecoveryStatus'."""

    __slots__ = [
        '_robot_id',
        '_recovery_type',
        '_degraded_capabilities',
        '_infeasible_tasks',
        '_timestamp',
    ]

    _fields_and_field_types = {
        'robot_id': 'int32',
        'recovery_type': 'string',
        'degraded_capabilities': 'sequence<float>',
        'infeasible_tasks': 'sequence<int32>',
        'timestamp': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', int())
        self.recovery_type = kwargs.get('recovery_type', str())
        self.degraded_capabilities = array.array('f', kwargs.get('degraded_capabilities', []))
        self.infeasible_tasks = array.array('i', kwargs.get('infeasible_tasks', []))
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
        if self.robot_id != other.robot_id:
            return False
        if self.recovery_type != other.recovery_type:
            return False
        if self.degraded_capabilities != other.degraded_capabilities:
            return False
        if self.infeasible_tasks != other.infeasible_tasks:
            return False
        if self.timestamp != other.timestamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'robot_id' field must be an integer in [-2147483648, 2147483647]"
        self._robot_id = value

    @builtins.property
    def recovery_type(self):
        """Message field 'recovery_type'."""
        return self._recovery_type

    @recovery_type.setter
    def recovery_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'recovery_type' field must be of type 'str'"
        self._recovery_type = value

    @builtins.property
    def degraded_capabilities(self):
        """Message field 'degraded_capabilities'."""
        return self._degraded_capabilities

    @degraded_capabilities.setter
    def degraded_capabilities(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'degraded_capabilities' array.array() must have the type code of 'f'"
            self._degraded_capabilities = value
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
                "The 'degraded_capabilities' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._degraded_capabilities = array.array('f', value)

    @builtins.property
    def infeasible_tasks(self):
        """Message field 'infeasible_tasks'."""
        return self._infeasible_tasks

    @infeasible_tasks.setter
    def infeasible_tasks(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'infeasible_tasks' array.array() must have the type code of 'i'"
            self._infeasible_tasks = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'infeasible_tasks' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._infeasible_tasks = array.array('i', value)

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
