# generated from rosidl_generator_py/resource/_idl.py.em
# with input from moving_table_interfaces:srv/MovingTable.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MovingTable_Request(type):
    """Metaclass of message 'MovingTable_Request'."""

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
            module = import_type_support('moving_table_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'moving_table_interfaces.srv.MovingTable_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__moving_table__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__moving_table__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__moving_table__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__moving_table__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__moving_table__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MovingTable_Request(metaclass=Metaclass_MovingTable_Request):
    """Message class 'MovingTable_Request'."""

    __slots__ = [
        '_table_id',
        '_distance_mm',
        '_angle_deg',
        '_linear_speed',
        '_rotate_speed',
        '_operation_type',
    ]

    _fields_and_field_types = {
        'table_id': 'string',
        'distance_mm': 'float',
        'angle_deg': 'float',
        'linear_speed': 'int32',
        'rotate_speed': 'int32',
        'operation_type': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.table_id = kwargs.get('table_id', str())
        self.distance_mm = kwargs.get('distance_mm', float())
        self.angle_deg = kwargs.get('angle_deg', float())
        self.linear_speed = kwargs.get('linear_speed', int())
        self.rotate_speed = kwargs.get('rotate_speed', int())
        self.operation_type = kwargs.get('operation_type', int())

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
        if self.table_id != other.table_id:
            return False
        if self.distance_mm != other.distance_mm:
            return False
        if self.angle_deg != other.angle_deg:
            return False
        if self.linear_speed != other.linear_speed:
            return False
        if self.rotate_speed != other.rotate_speed:
            return False
        if self.operation_type != other.operation_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def table_id(self):
        """Message field 'table_id'."""
        return self._table_id

    @table_id.setter
    def table_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'table_id' field must be of type 'str'"
        self._table_id = value

    @builtins.property
    def distance_mm(self):
        """Message field 'distance_mm'."""
        return self._distance_mm

    @distance_mm.setter
    def distance_mm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_mm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_mm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_mm = value

    @builtins.property
    def angle_deg(self):
        """Message field 'angle_deg'."""
        return self._angle_deg

    @angle_deg.setter
    def angle_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_deg = value

    @builtins.property
    def linear_speed(self):
        """Message field 'linear_speed'."""
        return self._linear_speed

    @linear_speed.setter
    def linear_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'linear_speed' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'linear_speed' field must be an integer in [-2147483648, 2147483647]"
        self._linear_speed = value

    @builtins.property
    def rotate_speed(self):
        """Message field 'rotate_speed'."""
        return self._rotate_speed

    @rotate_speed.setter
    def rotate_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rotate_speed' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rotate_speed' field must be an integer in [-2147483648, 2147483647]"
        self._rotate_speed = value

    @builtins.property
    def operation_type(self):
        """Message field 'operation_type'."""
        return self._operation_type

    @operation_type.setter
    def operation_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'operation_type' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'operation_type' field must be an integer in [-2147483648, 2147483647]"
        self._operation_type = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MovingTable_Response(type):
    """Metaclass of message 'MovingTable_Response'."""

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
            module = import_type_support('moving_table_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'moving_table_interfaces.srv.MovingTable_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__moving_table__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__moving_table__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__moving_table__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__moving_table__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__moving_table__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MovingTable_Response(metaclass=Metaclass_MovingTable_Response):
    """Message class 'MovingTable_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_MovingTable(type):
    """Metaclass of service 'MovingTable'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('moving_table_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'moving_table_interfaces.srv.MovingTable')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__moving_table

            from moving_table_interfaces.srv import _moving_table
            if _moving_table.Metaclass_MovingTable_Request._TYPE_SUPPORT is None:
                _moving_table.Metaclass_MovingTable_Request.__import_type_support__()
            if _moving_table.Metaclass_MovingTable_Response._TYPE_SUPPORT is None:
                _moving_table.Metaclass_MovingTable_Response.__import_type_support__()


class MovingTable(metaclass=Metaclass_MovingTable):
    from moving_table_interfaces.srv._moving_table import MovingTable_Request as Request
    from moving_table_interfaces.srv._moving_table import MovingTable_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
