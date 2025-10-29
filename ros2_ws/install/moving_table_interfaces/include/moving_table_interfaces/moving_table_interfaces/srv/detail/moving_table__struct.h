// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from moving_table_interfaces:srv/MovingTable.idl
// generated code does not contain a copyright notice

#ifndef MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__STRUCT_H_
#define MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'table_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MovingTable in the package moving_table_interfaces.
typedef struct moving_table_interfaces__srv__MovingTable_Request
{
  /// Add this field: Target table ("table1" or "table2")
  rosidl_runtime_c__String table_id;
  /// Target linear position (if applicable for operation_type)
  float distance_mm;
  /// Target angular position (if applicable for operation_type)
  float angle_deg;
  /// Speed for linear movement
  int32_t linear_speed;
  /// Speed for rotational movement
  int32_t rotate_speed;
  /// Defines which specific action to perform (e.g., move linear, move rotate)
  int32_t operation_type;
} moving_table_interfaces__srv__MovingTable_Request;

// Struct for a sequence of moving_table_interfaces__srv__MovingTable_Request.
typedef struct moving_table_interfaces__srv__MovingTable_Request__Sequence
{
  moving_table_interfaces__srv__MovingTable_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moving_table_interfaces__srv__MovingTable_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MovingTable in the package moving_table_interfaces.
typedef struct moving_table_interfaces__srv__MovingTable_Response
{
  /// True if the command was accepted and completed/started
  bool success;
  /// Feedback message (e.g., "OK", "Invalid table_id", "Movement started")
  rosidl_runtime_c__String message;
} moving_table_interfaces__srv__MovingTable_Response;

// Struct for a sequence of moving_table_interfaces__srv__MovingTable_Response.
typedef struct moving_table_interfaces__srv__MovingTable_Response__Sequence
{
  moving_table_interfaces__srv__MovingTable_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moving_table_interfaces__srv__MovingTable_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__STRUCT_H_
