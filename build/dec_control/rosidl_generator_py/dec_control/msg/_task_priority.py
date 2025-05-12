# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dec_control:msg/TaskPriority.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'task_ids'
# Member 'priorities'
# Member 'slack_times'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TaskPriority(type):
    """Metaclass of message 'TaskPriority'."""

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
                'dec_control.msg.TaskPriority')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__task_priority
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__task_priority
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__task_priority
            cls._TYPE_SUPPORT = module.type_support_msg__msg__task_priority
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__task_priority

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TaskPriority(metaclass=Metaclass_TaskPriority):
    """Message class 'TaskPriority'."""

    __slots__ = [
        '_task_ids',
        '_priorities',
        '_on_critical_path',
        '_slack_times',
    ]

    _fields_and_field_types = {
        'task_ids': 'sequence<int32>',
        'priorities': 'sequence<double>',
        'on_critical_path': 'sequence<boolean>',
        'slack_times': 'sequence<double>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.task_ids = array.array('i', kwargs.get('task_ids', []))
        self.priorities = array.array('d', kwargs.get('priorities', []))
        self.on_critical_path = kwargs.get('on_critical_path', [])
        self.slack_times = array.array('d', kwargs.get('slack_times', []))

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
        if self.task_ids != other.task_ids:
            return False
        if self.priorities != other.priorities:
            return False
        if self.on_critical_path != other.on_critical_path:
            return False
        if self.slack_times != other.slack_times:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def task_ids(self):
        """Message field 'task_ids'."""
        return self._task_ids

    @task_ids.setter
    def task_ids(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'task_ids' array.array() must have the type code of 'i'"
            self._task_ids = value
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
                "The 'task_ids' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._task_ids = array.array('i', value)

    @builtins.property
    def priorities(self):
        """Message field 'priorities'."""
        return self._priorities

    @priorities.setter
    def priorities(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'priorities' array.array() must have the type code of 'd'"
            self._priorities = value
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
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'priorities' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._priorities = array.array('d', value)

    @builtins.property
    def on_critical_path(self):
        """Message field 'on_critical_path'."""
        return self._on_critical_path

    @on_critical_path.setter
    def on_critical_path(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'on_critical_path' field must be a set or sequence and each value of type 'bool'"
        self._on_critical_path = value

    @builtins.property
    def slack_times(self):
        """Message field 'slack_times'."""
        return self._slack_times

    @slack_times.setter
    def slack_times(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'slack_times' array.array() must have the type code of 'd'"
            self._slack_times = value
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
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'slack_times' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._slack_times = array.array('d', value)
