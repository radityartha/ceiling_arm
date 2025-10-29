// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from moving_table_interfaces:srv/MovingTable.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "moving_table_interfaces/srv/detail/moving_table__rosidl_typesupport_introspection_c.h"
#include "moving_table_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "moving_table_interfaces/srv/detail/moving_table__functions.h"
#include "moving_table_interfaces/srv/detail/moving_table__struct.h"


// Include directives for member types
// Member `table_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  moving_table_interfaces__srv__MovingTable_Request__init(message_memory);
}

void moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_fini_function(void * message_memory)
{
  moving_table_interfaces__srv__MovingTable_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_member_array[6] = {
  {
    "table_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Request, table_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_mm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Request, distance_mm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Request, angle_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Request, linear_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rotate_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Request, rotate_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "operation_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Request, operation_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_members = {
  "moving_table_interfaces__srv",  // message namespace
  "MovingTable_Request",  // message name
  6,  // number of fields
  sizeof(moving_table_interfaces__srv__MovingTable_Request),
  moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_member_array,  // message members
  moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_type_support_handle = {
  0,
  &moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moving_table_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moving_table_interfaces, srv, MovingTable_Request)() {
  if (!moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_type_support_handle.typesupport_identifier) {
    moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &moving_table_interfaces__srv__MovingTable_Request__rosidl_typesupport_introspection_c__MovingTable_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "moving_table_interfaces/srv/detail/moving_table__rosidl_typesupport_introspection_c.h"
// already included above
// #include "moving_table_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "moving_table_interfaces/srv/detail/moving_table__functions.h"
// already included above
// #include "moving_table_interfaces/srv/detail/moving_table__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  moving_table_interfaces__srv__MovingTable_Response__init(message_memory);
}

void moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_fini_function(void * message_memory)
{
  moving_table_interfaces__srv__MovingTable_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moving_table_interfaces__srv__MovingTable_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_members = {
  "moving_table_interfaces__srv",  // message namespace
  "MovingTable_Response",  // message name
  2,  // number of fields
  sizeof(moving_table_interfaces__srv__MovingTable_Response),
  moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_member_array,  // message members
  moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_type_support_handle = {
  0,
  &moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moving_table_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moving_table_interfaces, srv, MovingTable_Response)() {
  if (!moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_type_support_handle.typesupport_identifier) {
    moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &moving_table_interfaces__srv__MovingTable_Response__rosidl_typesupport_introspection_c__MovingTable_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "moving_table_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "moving_table_interfaces/srv/detail/moving_table__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_service_members = {
  "moving_table_interfaces__srv",  // service namespace
  "MovingTable",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_Request_message_type_support_handle,
  NULL  // response message
  // moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_Response_message_type_support_handle
};

static rosidl_service_type_support_t moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_service_type_support_handle = {
  0,
  &moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moving_table_interfaces, srv, MovingTable_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moving_table_interfaces, srv, MovingTable_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moving_table_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moving_table_interfaces, srv, MovingTable)() {
  if (!moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_service_type_support_handle.typesupport_identifier) {
    moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moving_table_interfaces, srv, MovingTable_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moving_table_interfaces, srv, MovingTable_Response)()->data;
  }

  return &moving_table_interfaces__srv__detail__moving_table__rosidl_typesupport_introspection_c__MovingTable_service_type_support_handle;
}
