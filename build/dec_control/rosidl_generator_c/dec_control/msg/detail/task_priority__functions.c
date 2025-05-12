// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/task_priority__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `task_ids`
// Member `priorities`
// Member `on_critical_path`
// Member `slack_times`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dec_control__msg__TaskPriority__init(dec_control__msg__TaskPriority * msg)
{
  if (!msg) {
    return false;
  }
  // task_ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->task_ids, 0)) {
    dec_control__msg__TaskPriority__fini(msg);
    return false;
  }
  // priorities
  if (!rosidl_runtime_c__double__Sequence__init(&msg->priorities, 0)) {
    dec_control__msg__TaskPriority__fini(msg);
    return false;
  }
  // on_critical_path
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->on_critical_path, 0)) {
    dec_control__msg__TaskPriority__fini(msg);
    return false;
  }
  // slack_times
  if (!rosidl_runtime_c__double__Sequence__init(&msg->slack_times, 0)) {
    dec_control__msg__TaskPriority__fini(msg);
    return false;
  }
  return true;
}

void
dec_control__msg__TaskPriority__fini(dec_control__msg__TaskPriority * msg)
{
  if (!msg) {
    return;
  }
  // task_ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->task_ids);
  // priorities
  rosidl_runtime_c__double__Sequence__fini(&msg->priorities);
  // on_critical_path
  rosidl_runtime_c__boolean__Sequence__fini(&msg->on_critical_path);
  // slack_times
  rosidl_runtime_c__double__Sequence__fini(&msg->slack_times);
}

bool
dec_control__msg__TaskPriority__are_equal(const dec_control__msg__TaskPriority * lhs, const dec_control__msg__TaskPriority * rhs)
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
  // priorities
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->priorities), &(rhs->priorities)))
  {
    return false;
  }
  // on_critical_path
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->on_critical_path), &(rhs->on_critical_path)))
  {
    return false;
  }
  // slack_times
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->slack_times), &(rhs->slack_times)))
  {
    return false;
  }
  return true;
}

bool
dec_control__msg__TaskPriority__copy(
  const dec_control__msg__TaskPriority * input,
  dec_control__msg__TaskPriority * output)
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
  // priorities
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->priorities), &(output->priorities)))
  {
    return false;
  }
  // on_critical_path
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->on_critical_path), &(output->on_critical_path)))
  {
    return false;
  }
  // slack_times
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->slack_times), &(output->slack_times)))
  {
    return false;
  }
  return true;
}

dec_control__msg__TaskPriority *
dec_control__msg__TaskPriority__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__TaskPriority * msg = (dec_control__msg__TaskPriority *)allocator.allocate(sizeof(dec_control__msg__TaskPriority), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__msg__TaskPriority));
  bool success = dec_control__msg__TaskPriority__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__msg__TaskPriority__destroy(dec_control__msg__TaskPriority * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__msg__TaskPriority__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__msg__TaskPriority__Sequence__init(dec_control__msg__TaskPriority__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__TaskPriority * data = NULL;

  if (size) {
    data = (dec_control__msg__TaskPriority *)allocator.zero_allocate(size, sizeof(dec_control__msg__TaskPriority), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__msg__TaskPriority__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__msg__TaskPriority__fini(&data[i - 1]);
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
dec_control__msg__TaskPriority__Sequence__fini(dec_control__msg__TaskPriority__Sequence * array)
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
      dec_control__msg__TaskPriority__fini(&array->data[i]);
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

dec_control__msg__TaskPriority__Sequence *
dec_control__msg__TaskPriority__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__TaskPriority__Sequence * array = (dec_control__msg__TaskPriority__Sequence *)allocator.allocate(sizeof(dec_control__msg__TaskPriority__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__msg__TaskPriority__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__msg__TaskPriority__Sequence__destroy(dec_control__msg__TaskPriority__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__msg__TaskPriority__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__msg__TaskPriority__Sequence__are_equal(const dec_control__msg__TaskPriority__Sequence * lhs, const dec_control__msg__TaskPriority__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__msg__TaskPriority__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__msg__TaskPriority__Sequence__copy(
  const dec_control__msg__TaskPriority__Sequence * input,
  dec_control__msg__TaskPriority__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__msg__TaskPriority);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__msg__TaskPriority * data =
      (dec_control__msg__TaskPriority *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__msg__TaskPriority__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__msg__TaskPriority__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__msg__TaskPriority__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
