// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from moving_table_interfaces:srv/MovingTable.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "moving_table_interfaces/srv/detail/moving_table__struct.h"
#include "moving_table_interfaces/srv/detail/moving_table__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool moving_table_interfaces__srv__moving_table__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[62];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("moving_table_interfaces.srv._moving_table.MovingTable_Request", full_classname_dest, 61) == 0);
  }
  moving_table_interfaces__srv__MovingTable_Request * ros_message = _ros_message;
  {  // table_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "table_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->table_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // distance_mm
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_mm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_mm = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angle_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // linear_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_speed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->linear_speed = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rotate_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "rotate_speed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rotate_speed = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // operation_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "operation_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->operation_type = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * moving_table_interfaces__srv__moving_table__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MovingTable_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("moving_table_interfaces.srv._moving_table");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MovingTable_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  moving_table_interfaces__srv__MovingTable_Request * ros_message = (moving_table_interfaces__srv__MovingTable_Request *)raw_ros_message;
  {  // table_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->table_id.data,
      strlen(ros_message->table_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "table_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_mm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_mm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_mm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_speed
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->linear_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rotate_speed
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rotate_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rotate_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operation_type
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->operation_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operation_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "moving_table_interfaces/srv/detail/moving_table__struct.h"
// already included above
// #include "moving_table_interfaces/srv/detail/moving_table__functions.h"

// already included above
// #include "rosidl_runtime_c/string.h"
// already included above
// #include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool moving_table_interfaces__srv__moving_table__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[63];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("moving_table_interfaces.srv._moving_table.MovingTable_Response", full_classname_dest, 62) == 0);
  }
  moving_table_interfaces__srv__MovingTable_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * moving_table_interfaces__srv__moving_table__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MovingTable_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("moving_table_interfaces.srv._moving_table");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MovingTable_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  moving_table_interfaces__srv__MovingTable_Response * ros_message = (moving_table_interfaces__srv__MovingTable_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
