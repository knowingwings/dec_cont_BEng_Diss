# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dec_control:srv/InitAuction.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'capabilities'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InitAuction_Request(type):
    """Metaclass of message 'InitAuction_Request'."""

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
                'dec_control.srv.InitAuction_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__init_auction__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__init_auction__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__init_auction__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__init_auction__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__init_auction__request

            from dec_control.msg import Task
            if Task.__class__._TYPE_SUPPORT is None:
                Task.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InitAuction_Request(metaclass=Metaclass_InitAuction_Request):
    """Message class 'InitAuction_Request'."""

    __slots__ = [
        '_task',
        '_capabilities',
    ]

    _fields_and_field_types = {
        'task': 'dec_control/Task',
        'capabilities': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['dec_control', 'msg'], 'Task'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from dec_control.msg import Task
        self.task = kwargs.get('task', Task())
        self.capabilities = array.array('f', kwargs.get('capabilities', []))

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
        if self.task != other.task:
            return False
        if self.capabilities != other.capabilities:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def task(self):
        """Message field 'task'."""
        return self._task

    @task.setter
    def task(self, value):
        if __debug__:
            from dec_control.msg import Task
            assert \
                isinstance(value, Task), \
                "The 'task' field must be a sub message of type 'Task'"
        self._task = value

    @builtins.property
    def capabilities(self):
        """Message field 'capabilities'."""
        return self._capabilities

    @capabilities.setter
    def capabilities(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'capabilities' array.array() must have the type code of 'f'"
            self._capabilities = value
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
                "The 'capabilities' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._capabilities = array.array('f', value)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_InitAuction_Response(type):
    """Metaclass of message 'InitAuction_Response'."""

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
                'dec_control.srv.InitAuction_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__init_auction__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__init_auction__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__init_auction__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__init_auction__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__init_auction__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InitAuction_Response(metaclass=Metaclass_InitAuction_Response):
    """Message class 'InitAuction_Response'."""

    __slots__ = [
        '_auction_id',
        '_success',
    ]

    _fields_and_field_types = {
        'auction_id': 'string',
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.auction_id = kwargs.get('auction_id', str())
        self.success = kwargs.get('success', bool())

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
        if self.auction_id != other.auction_id:
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def auction_id(self):
        """Message field 'auction_id'."""
        return self._auction_id

    @auction_id.setter
    def auction_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'auction_id' field must be of type 'str'"
        self._auction_id = value

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_InitAuction(type):
    """Metaclass of service 'InitAuction'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dec_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dec_control.srv.InitAuction')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__init_auction

            from dec_control.srv import _init_auction
            if _init_auction.Metaclass_InitAuction_Request._TYPE_SUPPORT is None:
                _init_auction.Metaclass_InitAuction_Request.__import_type_support__()
            if _init_auction.Metaclass_InitAuction_Response._TYPE_SUPPORT is None:
                _init_auction.Metaclass_InitAuction_Response.__import_type_support__()


class InitAuction(metaclass=Metaclass_InitAuction):
    from dec_control.srv._init_auction import InitAuction_Request as Request
    from dec_control.srv._init_auction import InitAuction_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
