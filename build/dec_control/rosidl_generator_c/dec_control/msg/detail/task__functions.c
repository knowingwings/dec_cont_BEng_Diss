// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:msg/Task.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `capabilities_required`
// Member `prerequisites`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dec_control__msg__Task__init(dec_control__msg__Task * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // position
  // capabilities_required
  if (!rosidl_runtime_c__float__Sequence__init(&msg->capabilities_required, 0)) {
    dec_control__msg__Task__fini(msg);
    return false;
  }
  // execution_time
  // prerequisites
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->prerequisites, 0)) {
    dec_control__msg__Task__fini(msg);
    return false;
  }
  // requires_collaboration
  return true;
}

void
dec_control__msg__Task__fini(dec_control__msg__Task * msg)
{
  if (!msg) {
    return;
  }
  // id
  // position
  // capabilities_required
  rosidl_runtime_c__float__Sequence__fini(&msg->capabilities_required);
  // execution_time
  // prerequisites
  rosidl_runtime_c__int32__Sequence__fini(&msg->prerequisites);
  // requires_collaboration
}

bool
dec_control__msg__Task__are_equal(const dec_control__msg__Task * lhs, const dec_control__msg__Task * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // position
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->position[i] != rhs->position[i]) {
      return false;
    }
  }
  // capabilities_required
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->capabilities_required), &(rhs->capabilities_required)))
  {
    return false;
  }
  // execution_time
  if (lhs->execution_time != rhs->execution_time) {
    return false;
  }
  // prerequisites
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->prerequisites), &(rhs->prerequisites)))
  {
    return false;
  }
  // requires_collaboration
  if (lhs->requires_collaboration != rhs->requires_collaboration) {
    return false;
  }
  return true;
}

bool
dec_control__msg__Task__copy(
  const dec_control__msg__Task * input,
  dec_control__msg__Task * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // position
  for (size_t i = 0; i < 3; ++i) {
    output->position[i] = input->position[i];
  }
  // capabilities_required
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->capabilities_required), &(output->capabilities_required)))
  {
    return false;
  }
  // execution_time
  output->execution_time = input->execution_time;
  // prerequisites
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->prerequisites), &(output->prerequisites)))
  {
    return false;
  }
  // requires_collaboration
  output->requires_collaboration = input->requires_collaboration;
  return true;
}

dec_control__msg__Task *
dec_control__msg__Task__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__Task * msg = (dec_control__msg__Task *)allocator.allocate(sizeof(dec_control__msg__Task), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__msg__Task));
  bool success = dec_control__msg__Task__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__msg__Task__destroy(dec_control__msg__Task * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__msg__Task__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__msg__Task__Sequence__init(dec_control__msg__Task__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__Task * data = NULL;

  if (size) {
    data = (dec_control__msg__Task *)allocator.zero_allocate(size, sizeof(dec_control__msg__Task), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__msg__Task__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__msg__Task__fini(&data[i - 1]);
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
dec_control__msg__Task__Sequence__fini(dec_control__msg__Task__Sequence * array)
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
      dec_control__msg__Task__fini(&array->data[i]);
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

dec_control__msg__Task__Sequence *
dec_control__msg__Task__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__Task__Sequence * array = (dec_control__msg__Task__Sequence *)allocator.allocate(sizeof(dec_control__msg__Task__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__msg__Task__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__msg__Task__Sequence__destroy(dec_control__msg__Task__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__msg__Task__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__msg__Task__Sequence__are_equal(const dec_control__msg__Task__Sequence * lhs, const dec_control__msg__Task__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__msg__Task__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__msg__Task__Sequence__copy(
  const dec_control__msg__Task__Sequence * input,
  dec_control__msg__Task__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__msg__Task);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__msg__Task * data =
      (dec_control__msg__Task *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__msg__Task__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__msg__Task__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__msg__Task__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
