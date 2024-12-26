# generated from rosidl_generator_py/resource/_idl.py.em
# with input from message_interfaces:msg/BoundingBox.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BoundingBox(type):
    """Metaclass of message 'BoundingBox'."""

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
            module = import_type_support('message_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'message_interfaces.msg.BoundingBox')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bounding_box
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bounding_box
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bounding_box
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bounding_box
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bounding_box

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BoundingBox(metaclass=Metaclass_BoundingBox):
    """Message class 'BoundingBox'."""

    __slots__ = [
        '_lu_x',
        '_lu_y',
        '_rd_x',
        '_rd_y',
    ]

    _fields_and_field_types = {
        'lu_x': 'int32',
        'lu_y': 'int32',
        'rd_x': 'int32',
        'rd_y': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.lu_x = kwargs.get('lu_x', int())
        self.lu_y = kwargs.get('lu_y', int())
        self.rd_x = kwargs.get('rd_x', int())
        self.rd_y = kwargs.get('rd_y', int())

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
        if self.lu_x != other.lu_x:
            return False
        if self.lu_y != other.lu_y:
            return False
        if self.rd_x != other.rd_x:
            return False
        if self.rd_y != other.rd_y:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def lu_x(self):
        """Message field 'lu_x'."""
        return self._lu_x

    @lu_x.setter
    def lu_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lu_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'lu_x' field must be an integer in [-2147483648, 2147483647]"
        self._lu_x = value

    @builtins.property
    def lu_y(self):
        """Message field 'lu_y'."""
        return self._lu_y

    @lu_y.setter
    def lu_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lu_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'lu_y' field must be an integer in [-2147483648, 2147483647]"
        self._lu_y = value

    @builtins.property
    def rd_x(self):
        """Message field 'rd_x'."""
        return self._rd_x

    @rd_x.setter
    def rd_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rd_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rd_x' field must be an integer in [-2147483648, 2147483647]"
        self._rd_x = value

    @builtins.property
    def rd_y(self):
        """Message field 'rd_y'."""
        return self._rd_y

    @rd_y.setter
    def rd_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rd_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rd_y' field must be an integer in [-2147483648, 2147483647]"
        self._rd_y = value
