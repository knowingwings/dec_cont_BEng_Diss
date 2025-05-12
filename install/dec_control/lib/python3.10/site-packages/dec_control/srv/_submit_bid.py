# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dec_control:srv/SubmitBid.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'resource_availability'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SubmitBid_Request(type):
    """Metaclass of message 'SubmitBid_Request'."""

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
                'dec_control.srv.SubmitBid_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__submit_bid__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__submit_bid__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__submit_bid__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__submit_bid__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__submit_bid__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SubmitBid_Request(metaclass=Metaclass_SubmitBid_Request):
    """Message class 'SubmitBid_Request'."""

    __slots__ = [
        '_auction_id',
        '_robot_id',
        '_bid_value',
        '_resource_availability',
    ]

    _fields_and_field_types = {
        'auction_id': 'string',
        'robot_id': 'string',
        'bid_value': 'float',
        'resource_availability': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.auction_id = kwargs.get('auction_id', str())
        self.robot_id = kwargs.get('robot_id', str())
        self.bid_value = kwargs.get('bid_value', float())
        self.resource_availability = array.array('f', kwargs.get('resource_availability', []))

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
        if self.robot_id != other.robot_id:
            return False
        if self.bid_value != other.bid_value:
            return False
        if self.resource_availability != other.resource_availability:
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
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_id' field must be of type 'str'"
        self._robot_id = value

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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bid_value' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bid_value = value

    @builtins.property
    def resource_availability(self):
        """Message field 'resource_availability'."""
        return self._resource_availability

    @resource_availability.setter
    def resource_availability(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'resource_availability' array.array() must have the type code of 'f'"
            self._resource_availability = value
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
                "The 'resource_availability' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._resource_availability = array.array('f', value)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SubmitBid_Response(type):
    """Metaclass of message 'SubmitBid_Response'."""

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
                'dec_control.srv.SubmitBid_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__submit_bid__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__submit_bid__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__submit_bid__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__submit_bid__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__submit_bid__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SubmitBid_Response(metaclass=Metaclass_SubmitBid_Response):
    """Message class 'SubmitBid_Response'."""

    __slots__ = [
        '_accepted',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())

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
        if self.accepted != other.accepted:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value


class Metaclass_SubmitBid(type):
    """Metaclass of service 'SubmitBid'."""

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
                'dec_control.srv.SubmitBid')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__submit_bid

            from dec_control.srv import _submit_bid
            if _submit_bid.Metaclass_SubmitBid_Request._TYPE_SUPPORT is None:
                _submit_bid.Metaclass_SubmitBid_Request.__import_type_support__()
            if _submit_bid.Metaclass_SubmitBid_Response._TYPE_SUPPORT is None:
                _submit_bid.Metaclass_SubmitBid_Response.__import_type_support__()


class SubmitBid(metaclass=Metaclass_SubmitBid):
    from dec_control.srv._submit_bid import SubmitBid_Request as Request
    from dec_control.srv._submit_bid import SubmitBid_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
