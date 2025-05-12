// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:msg/TaskAssignment.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/task_assignment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `task_ids`
// Member `robot_ids`
// Member `prices`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dec_control__msg__TaskAssignment__init(dec_control__msg__TaskAssignment * msg)
{
  if (!msg) {
    return false;
  }
  // task_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->task_ids, 0)) {
    dec_control__msg__TaskAssignment__fini(msg);
    return false;
  }
  // robot_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->robot_ids, 0)) {
    dec_control__msg__TaskAssignment__fini(msg);
    return false;
  }
  // prices
  if (!rosidl_runtime_c__double__Sequence__init(&msg->prices, 0)) {
    dec_control__msg__TaskAssignment__fini(msg);
    return false;
  }
  return true;
}

void
dec_control__msg__TaskAssignment__fini(dec_control__msg__TaskAssignment * msg)
{
  if (!msg) {
    return;
  }
  // task_ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->task_ids);
  // robot_ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->robot_ids);
  // prices
  rosidl_runtime_c__double__Sequence__fini(&msg->prices);
}

bool
dec_control__msg__TaskAssignment__are_equal(const dec_control__msg__TaskAssignment * lhs, const dec_control__msg__TaskAssignment * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // task_ids
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->task_ids), &(rhs->task_ids)))
  {
    return false;
  }
  // robot_ids
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->robot_ids), &(rhs->robot_ids)))
  {
    return false;
  }
  // prices
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->prices), &(rhs->prices)))
  {
    return false;
  }
  return true;
}

bool
dec_control__msg__TaskAssignment__copy(
  const dec_control__msg__TaskAssignment * input,
  dec_control__msg__TaskAssignment * output)
{
  if (!input || !output) {
    return false;
  }
  // task_ids
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->task_ids), &(output->task_ids)))
  {
    return false;
  }
  // robot_ids
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->robot_ids), &(output->robot_ids)))
  {
    return false;
  }
  // prices
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->prices), &(output->prices)))
  {
    return false;
  }
  return true;
}

dec_control__msg__TaskAssignment *
dec_control__msg__TaskAssignment__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__TaskAssignment * msg = (dec_control__msg__TaskAssignment *)allocator.allocate(sizeof(dec_control__msg__TaskAssignment), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__msg__TaskAssignment));
  bool success = dec_control__msg__TaskAssignment__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__msg__TaskAssignment__destroy(dec_control__msg__TaskAssignment * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__msg__TaskAssignment__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__msg__TaskAssignment__Sequence__init(dec_control__msg__TaskAssignment__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__TaskAssignment * data = NULL;

  if (size) {
    data = (dec_control__msg__TaskAssignment *)allocator.zero_allocate(size, sizeof(dec_control__msg__TaskAssignment), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__msg__TaskAssignment__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__msg__TaskAssignment__fini(&data[i - 1]);
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
dec_control__msg__TaskAssignment__Sequence__fini(dec_control__msg__TaskAssignment__Sequence * array)
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
      dec_control__msg__TaskAssignment__fini(&array->data[i]);
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

dec_control__msg__TaskAssignment__Sequence *
dec_control__msg__TaskAssignment__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__TaskAssignment__Sequence * array = (dec_control__msg__TaskAssignment__Sequence *)allocator.allocate(sizeof(dec_control__msg__TaskAssignment__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__msg__TaskAssignment__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__msg__TaskAssignment__Sequence__destroy(dec_control__msg__TaskAssignment__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__msg__TaskAssignment__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__msg__TaskAssignment__Sequence__are_equal(const dec_control__msg__TaskAssignment__Sequence * lhs, const dec_control__msg__TaskAssignment__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__msg__TaskAssignment__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__msg__TaskAssignment__Sequence__copy(
  const dec_control__msg__TaskAssignment__Sequence * input,
  dec_control__msg__TaskAssignment__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__msg__TaskAssignment);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__msg__TaskAssignment * data =
      (dec_control__msg__TaskAssignment *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__msg__TaskAssignment__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__msg__TaskAssignment__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__msg__TaskAssignment__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
