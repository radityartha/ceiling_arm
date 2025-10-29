// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from moving_table_interfaces:srv/MovingTable.idl
// generated code does not contain a copyright notice
#include "moving_table_interfaces/srv/detail/moving_table__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `table_id`
#include "rosidl_runtime_c/string_functions.h"

bool
moving_table_interfaces__srv__MovingTable_Request__init(moving_table_interfaces__srv__MovingTable_Request * msg)
{
  if (!msg) {
    return false;
  }
  // table_id
  if (!rosidl_runtime_c__String__init(&msg->table_id)) {
    moving_table_interfaces__srv__MovingTable_Request__fini(msg);
    return false;
  }
  // distance_mm
  // angle_deg
  // linear_speed
  // rotate_speed
  // operation_type
  return true;
}

void
moving_table_interfaces__srv__MovingTable_Request__fini(moving_table_interfaces__srv__MovingTable_Request * msg)
{
  if (!msg) {
    return;
  }
  // table_id
  rosidl_runtime_c__String__fini(&msg->table_id);
  // distance_mm
  // angle_deg
  // linear_speed
  // rotate_speed
  // operation_type
}

bool
moving_table_interfaces__srv__MovingTable_Request__are_equal(const moving_table_interfaces__srv__MovingTable_Request * lhs, const moving_table_interfaces__srv__MovingTable_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // table_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->table_id), &(rhs->table_id)))
  {
    return false;
  }
  // distance_mm
  if (lhs->distance_mm != rhs->distance_mm) {
    return false;
  }
  // angle_deg
  if (lhs->angle_deg != rhs->angle_deg) {
    return false;
  }
  // linear_speed
  if (lhs->linear_speed != rhs->linear_speed) {
    return false;
  }
  // rotate_speed
  if (lhs->rotate_speed != rhs->rotate_speed) {
    return false;
  }
  // operation_type
  if (lhs->operation_type != rhs->operation_type) {
    return false;
  }
  return true;
}

bool
moving_table_interfaces__srv__MovingTable_Request__copy(
  const moving_table_interfaces__srv__MovingTable_Request * input,
  moving_table_interfaces__srv__MovingTable_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // table_id
  if (!rosidl_runtime_c__String__copy(
      &(input->table_id), &(output->table_id)))
  {
    return false;
  }
  // distance_mm
  output->distance_mm = input->distance_mm;
  // angle_deg
  output->angle_deg = input->angle_deg;
  // linear_speed
  output->linear_speed = input->linear_speed;
  // rotate_speed
  output->rotate_speed = input->rotate_speed;
  // operation_type
  output->operation_type = input->operation_type;
  return true;
}

moving_table_interfaces__srv__MovingTable_Request *
moving_table_interfaces__srv__MovingTable_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moving_table_interfaces__srv__MovingTable_Request * msg = (moving_table_interfaces__srv__MovingTable_Request *)allocator.allocate(sizeof(moving_table_interfaces__srv__MovingTable_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(moving_table_interfaces__srv__MovingTable_Request));
  bool success = moving_table_interfaces__srv__MovingTable_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
moving_table_interfaces__srv__MovingTable_Request__destroy(moving_table_interfaces__srv__MovingTable_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    moving_table_interfaces__srv__MovingTable_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
moving_table_interfaces__srv__MovingTable_Request__Sequence__init(moving_table_interfaces__srv__MovingTable_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moving_table_interfaces__srv__MovingTable_Request * data = NULL;

  if (size) {
    data = (moving_table_interfaces__srv__MovingTable_Request *)allocator.zero_allocate(size, sizeof(moving_table_interfaces__srv__MovingTable_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = moving_table_interfaces__srv__MovingTable_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        moving_table_interfaces__srv__MovingTable_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
moving_table_interfaces__srv__MovingTable_Request__Sequence__fini(moving_table_interfaces__srv__MovingTable_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      moving_table_interfaces__srv__MovingTable_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

moving_table_interfaces__srv__MovingTable_Request__Sequence *
moving_table_interfaces__srv__MovingTable_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moving_table_interfaces__srv__MovingTable_Request__Sequence * array = (moving_table_interfaces__srv__MovingTable_Request__Sequence *)allocator.allocate(sizeof(moving_table_interfaces__srv__MovingTable_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = moving_table_interfaces__srv__MovingTable_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
moving_table_interfaces__srv__MovingTable_Request__Sequence__destroy(moving_table_interfaces__srv__MovingTable_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    moving_table_interfaces__srv__MovingTable_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
moving_table_interfaces__srv__MovingTable_Request__Sequence__are_equal(const moving_table_interfaces__srv__MovingTable_Request__Sequence * lhs, const moving_table_interfaces__srv__MovingTable_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!moving_table_interfaces__srv__MovingTable_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
moving_table_interfaces__srv__MovingTable_Request__Sequence__copy(
  const moving_table_interfaces__srv__MovingTable_Request__Sequence * input,
  moving_table_interfaces__srv__MovingTable_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(moving_table_interfaces__srv__MovingTable_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    moving_table_interfaces__srv__MovingTable_Request * data =
      (moving_table_interfaces__srv__MovingTable_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!moving_table_interfaces__srv__MovingTable_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          moving_table_interfaces__srv__MovingTable_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!moving_table_interfaces__srv__MovingTable_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
moving_table_interfaces__srv__MovingTable_Response__init(moving_table_interfaces__srv__MovingTable_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    moving_table_interfaces__srv__MovingTable_Response__fini(msg);
    return false;
  }
  return true;
}

void
moving_table_interfaces__srv__MovingTable_Response__fini(moving_table_interfaces__srv__MovingTable_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
moving_table_interfaces__srv__MovingTable_Response__are_equal(const moving_table_interfaces__srv__MovingTable_Response * lhs, const moving_table_interfaces__srv__MovingTable_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
moving_table_interfaces__srv__MovingTable_Response__copy(
  const moving_table_interfaces__srv__MovingTable_Response * input,
  moving_table_interfaces__srv__MovingTable_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

moving_table_interfaces__srv__MovingTable_Response *
moving_table_interfaces__srv__MovingTable_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moving_table_interfaces__srv__MovingTable_Response * msg = (moving_table_interfaces__srv__MovingTable_Response *)allocator.allocate(sizeof(moving_table_interfaces__srv__MovingTable_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(moving_table_interfaces__srv__MovingTable_Response));
  bool success = moving_table_interfaces__srv__MovingTable_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
moving_table_interfaces__srv__MovingTable_Response__destroy(moving_table_interfaces__srv__MovingTable_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    moving_table_interfaces__srv__MovingTable_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
moving_table_interfaces__srv__MovingTable_Response__Sequence__init(moving_table_interfaces__srv__MovingTable_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moving_table_interfaces__srv__MovingTable_Response * data = NULL;

  if (size) {
    data = (moving_table_interfaces__srv__MovingTable_Response *)allocator.zero_allocate(size, sizeof(moving_table_interfaces__srv__MovingTable_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = moving_table_interfaces__srv__MovingTable_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        moving_table_interfaces__srv__MovingTable_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
moving_table_interfaces__srv__MovingTable_Response__Sequence__fini(moving_table_interfaces__srv__MovingTable_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      moving_table_interfaces__srv__MovingTable_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

moving_table_interfaces__srv__MovingTable_Response__Sequence *
moving_table_interfaces__srv__MovingTable_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moving_table_interfaces__srv__MovingTable_Response__Sequence * array = (moving_table_interfaces__srv__MovingTable_Response__Sequence *)allocator.allocate(sizeof(moving_table_interfaces__srv__MovingTable_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = moving_table_interfaces__srv__MovingTable_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
moving_table_interfaces__srv__MovingTable_Response__Sequence__destroy(moving_table_interfaces__srv__MovingTable_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    moving_table_interfaces__srv__MovingTable_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
moving_table_interfaces__srv__MovingTable_Response__Sequence__are_equal(const moving_table_interfaces__srv__MovingTable_Response__Sequence * lhs, const moving_table_interfaces__srv__MovingTable_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!moving_table_interfaces__srv__MovingTable_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
moving_table_interfaces__srv__MovingTable_Response__Sequence__copy(
  const moving_table_interfaces__srv__MovingTable_Response__Sequence * input,
  moving_table_interfaces__srv__MovingTable_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(moving_table_interfaces__srv__MovingTable_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    moving_table_interfaces__srv__MovingTable_Response * data =
      (moving_table_interfaces__srv__MovingTable_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!moving_table_interfaces__srv__MovingTable_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          moving_table_interfaces__srv__MovingTable_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!moving_table_interfaces__srv__MovingTable_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
