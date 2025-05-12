// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/recovery_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `recovery_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `degraded_capabilities`
// Member `infeasible_tasks`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dec_control__msg__RecoveryStatus__init(dec_control__msg__RecoveryStatus * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // recovery_type
  if (!rosidl_runtime_c__String__init(&msg->recovery_type)) {
    dec_control__msg__RecoveryStatus__fini(msg);
    return false;
  }
  // degraded_capabilities
  if (!rosidl_runtime_c__float__Sequence__init(&msg->degraded_capabilities, 0)) {
    dec_control__msg__RecoveryStatus__fini(msg);
    return false;
  }
  // infeasible_tasks
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->infeasible_tasks, 0)) {
    dec_control__msg__RecoveryStatus__fini(msg);
    return false;
  }
  // timestamp
  return true;
}

void
dec_control__msg__RecoveryStatus__fini(dec_control__msg__RecoveryStatus * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // recovery_type
  rosidl_runtime_c__String__fini(&msg->recovery_type);
  // degraded_capabilities
  rosidl_runtime_c__float__Sequence__fini(&msg->degraded_capabilities);
  // infeasible_tasks
  rosidl_runtime_c__int32__Sequence__fini(&msg->infeasible_tasks);
  // timestamp
}

bool
dec_control__msg__RecoveryStatus__are_equal(const dec_control__msg__RecoveryStatus * lhs, const dec_control__msg__RecoveryStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // recovery_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->recovery_type), &(rhs->recovery_type)))
  {
    return false;
  }
  // degraded_capabilities
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->degraded_capabilities), &(rhs->degraded_capabilities)))
  {
    return false;
  }
  // infeasible_tasks
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->infeasible_tasks), &(rhs->infeasible_tasks)))
  {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  return true;
}

bool
dec_control__msg__RecoveryStatus__copy(
  const dec_control__msg__RecoveryStatus * input,
  dec_control__msg__RecoveryStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // recovery_type
  if (!rosidl_runtime_c__String__copy(
      &(input->recovery_type), &(output->recovery_type)))
  {
    return false;
  }
  // degraded_capabilities
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->degraded_capabilities), &(output->degraded_capabilities)))
  {
    return false;
  }
  // infeasible_tasks
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->infeasible_tasks), &(output->infeasible_tasks)))
  {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  return true;
}

dec_control__msg__RecoveryStatus *
dec_control__msg__RecoveryStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__RecoveryStatus * msg = (dec_control__msg__RecoveryStatus *)allocator.allocate(sizeof(dec_control__msg__RecoveryStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__msg__RecoveryStatus));
  bool success = dec_control__msg__RecoveryStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__msg__RecoveryStatus__destroy(dec_control__msg__RecoveryStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__msg__RecoveryStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__msg__RecoveryStatus__Sequence__init(dec_control__msg__RecoveryStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__RecoveryStatus * data = NULL;

  if (size) {
    data = (dec_control__msg__RecoveryStatus *)allocator.zero_allocate(size, sizeof(dec_control__msg__RecoveryStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__msg__RecoveryStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__msg__RecoveryStatus__fini(&data[i - 1]);
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
dec_control__msg__RecoveryStatus__Sequence__fini(dec_control__msg__RecoveryStatus__Sequence * array)
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
      dec_control__msg__RecoveryStatus__fini(&array->data[i]);
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

dec_control__msg__RecoveryStatus__Sequence *
dec_control__msg__RecoveryStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__RecoveryStatus__Sequence * array = (dec_control__msg__RecoveryStatus__Sequence *)allocator.allocate(sizeof(dec_control__msg__RecoveryStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__msg__RecoveryStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__msg__RecoveryStatus__Sequence__destroy(dec_control__msg__RecoveryStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__msg__RecoveryStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__msg__RecoveryStatus__Sequence__are_equal(const dec_control__msg__RecoveryStatus__Sequence * lhs, const dec_control__msg__RecoveryStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__msg__RecoveryStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__msg__RecoveryStatus__Sequence__copy(
  const dec_control__msg__RecoveryStatus__Sequence * input,
  dec_control__msg__RecoveryStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__msg__RecoveryStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__msg__RecoveryStatus * data =
      (dec_control__msg__RecoveryStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__msg__RecoveryStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__msg__RecoveryStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__msg__RecoveryStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
