# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dec_control:msg/Bid.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Bid(type):
    """Metaclass of message 'Bid'."""

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
                'dec_control.msg.Bid')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bid
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bid
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bid
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bid
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bid

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Bid(metaclass=Metaclass_Bid):
    """Message class 'Bid'."""

    __slots__ = [
        '_robot_id',
        '_task_id',
        '_bid_value',
        '_utility',
    ]

    _fields_and_field_types = {
        'robot_id': 'int32',
        'task_id': 'int32',
        'bid_value': 'double',
        'utility': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', int())
        self.task_id = kwargs.get('task_id', int())
        self.bid_value = kwargs.get('bid_value', float())
        self.utility = kwargs.get('utility', float())

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
        if self.task_id != other.task_id:
            return False
        if self.bid_value != other.bid_value:
            return False
        if self.utility != other.utility:
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
    def task_id(self):
        """Message field 'task_id'."""
        return self._task_id

    @task_id.setter
    def task_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'task_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'task_id' field must be an integer in [-2147483648, 2147483647]"
        self._task_id = value

    @builtins.property
    def bid_value(self):
        """Message field 'bid_value'."""
        return self._bid_value

    @bid_value.setter
    def bid_value(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bid_value' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bid_value' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bid_value = value

    @builtins.property
    def utility(self):
        """Message field 'utility'."""
        return self._utility

    @utility.setter
    def utility(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'utility' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'utility' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._utility = value
