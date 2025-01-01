# generated from rosidl_generator_py/resource/_idl.py.em
# with input from message_interfaces:msg/NodeStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NodeStatus(type):
    """Metaclass of message 'NodeStatus'."""

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
                'message_interfaces.msg.NodeStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__node_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__node_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__node_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__node_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__node_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NodeStatus(metaclass=Metaclass_NodeStatus):
    """Message class 'NodeStatus'."""

    __slots__ = [
        '_is_moving',
        '_iteration',
        '_detection_done',
        '_icp_done',
        '_octomap_done',
        '_nbv_done',
        '_nbv_point_x',
        '_nbv_point_y',
        '_nbv_point_z',
        '_is_final_result',
    ]

    _fields_and_field_types = {
        'is_moving': 'boolean',
        'iteration': 'int32',
        'detection_done': 'boolean',
        'icp_done': 'boolean',
        'octomap_done': 'boolean',
        'nbv_done': 'boolean',
        'nbv_point_x': 'double',
        'nbv_point_y': 'double',
        'nbv_point_z': 'double',
        'is_final_result': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_moving = kwargs.get('is_moving', bool())
        self.iteration = kwargs.get('iteration', int())
        self.detection_done = kwargs.get('detection_done', bool())
        self.icp_done = kwargs.get('icp_done', bool())
        self.octomap_done = kwargs.get('octomap_done', bool())
        self.nbv_done = kwargs.get('nbv_done', bool())
        self.nbv_point_x = kwargs.get('nbv_point_x', float())
        self.nbv_point_y = kwargs.get('nbv_point_y', float())
        self.nbv_point_z = kwargs.get('nbv_point_z', float())
        self.is_final_result = kwargs.get('is_final_result', bool())

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
        if self.is_moving != other.is_moving:
            return False
        if self.iteration != other.iteration:
            return False
        if self.detection_done != other.detection_done:
            return False
        if self.icp_done != other.icp_done:
            return False
        if self.octomap_done != other.octomap_done:
            return False
        if self.nbv_done != other.nbv_done:
            return False
        if self.nbv_point_x != other.nbv_point_x:
            return False
        if self.nbv_point_y != other.nbv_point_y:
            return False
        if self.nbv_point_z != other.nbv_point_z:
            return False
        if self.is_final_result != other.is_final_result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_moving(self):
        """Message field 'is_moving'."""
        return self._is_moving

    @is_moving.setter
    def is_moving(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_moving' field must be of type 'bool'"
        self._is_moving = value

    @builtins.property
    def iteration(self):
        """Message field 'iteration'."""
        return self._iteration

    @iteration.setter
    def iteration(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'iteration' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'iteration' field must be an integer in [-2147483648, 2147483647]"
        self._iteration = value

    @builtins.property
    def detection_done(self):
        """Message field 'detection_done'."""
        return self._detection_done

    @detection_done.setter
    def detection_done(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'detection_done' field must be of type 'bool'"
        self._detection_done = value

    @builtins.property
    def icp_done(self):
        """Message field 'icp_done'."""
        return self._icp_done

    @icp_done.setter
    def icp_done(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'icp_done' field must be of type 'bool'"
        self._icp_done = value

    @builtins.property
    def octomap_done(self):
        """Message field 'octomap_done'."""
        return self._octomap_done

    @octomap_done.setter
    def octomap_done(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'octomap_done' field must be of type 'bool'"
        self._octomap_done = value

    @builtins.property
    def nbv_done(self):
        """Message field 'nbv_done'."""
        return self._nbv_done

    @nbv_done.setter
    def nbv_done(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'nbv_done' field must be of type 'bool'"
        self._nbv_done = value

    @builtins.property
    def nbv_point_x(self):
        """Message field 'nbv_point_x'."""
        return self._nbv_point_x

    @nbv_point_x.setter
    def nbv_point_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'nbv_point_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'nbv_point_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._nbv_point_x = value

    @builtins.property
    def nbv_point_y(self):
        """Message field 'nbv_point_y'."""
        return self._nbv_point_y

    @nbv_point_y.setter
    def nbv_point_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'nbv_point_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'nbv_point_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._nbv_point_y = value

    @builtins.property
    def nbv_point_z(self):
        """Message field 'nbv_point_z'."""
        return self._nbv_point_z

    @nbv_point_z.setter
    def nbv_point_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'nbv_point_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'nbv_point_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._nbv_point_z = value

    @builtins.property
    def is_final_result(self):
        """Message field 'is_final_result'."""
        return self._is_final_result

    @is_final_result.setter
    def is_final_result(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_final_result' field must be of type 'bool'"
        self._is_final_result = value
