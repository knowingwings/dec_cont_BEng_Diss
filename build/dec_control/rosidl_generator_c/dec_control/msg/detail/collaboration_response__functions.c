// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:msg/CollaborationResponse.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/collaboration_response__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
dec_control__msg__CollaborationResponse__init(dec_control__msg__CollaborationResponse * msg)
{
  if (!msg) {
    return false;
  }
  // task_id
  // robot_id
  // agree_to_role
  // is_leader
  // timestamp
  return true;
}

void
dec_control__msg__CollaborationResponse__fini(dec_control__msg__CollaborationResponse * msg)
{
  if (!msg) {
    return;
  }
  // task_id
  // robot_id
  // agree_to_role
  // is_leader
  // timestamp
}

bool
dec_control__msg__CollaborationResponse__are_equal(const dec_control__msg__CollaborationResponse * lhs, const dec_control__msg__CollaborationResponse * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // task_id
  if (lhs->task_id != rhs->task_id) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // agree_to_role
  if (lhs->agree_to_role != rhs->agree_to_role) {
    return false;
  }
  // is_leader
  if (lhs->is_leader != rhs->is_leader) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  return true;
}

bool
dec_control__msg__CollaborationResponse__copy(
  const dec_control__msg__CollaborationResponse * input,
  dec_control__msg__CollaborationResponse * output)
{
  if (!input || !output) {
    return false;
  }
  // task_id
  output->task_id = input->task_id;
  // robot_id
  output->robot_id = input->robot_id;
  // agree_to_role
  output->agree_to_role = input->agree_to_role;
  // is_leader
  output->is_leader = input->is_leader;
  // timestamp
  output->timestamp = input->timestamp;
  return true;
}

dec_control__msg__CollaborationResponse *
dec_control__msg__CollaborationResponse__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__CollaborationResponse * msg = (dec_control__msg__CollaborationResponse *)allocator.allocate(sizeof(dec_control__msg__CollaborationResponse), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__msg__CollaborationResponse));
  bool success = dec_control__msg__CollaborationResponse__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__msg__CollaborationResponse__destroy(dec_control__msg__CollaborationResponse * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__msg__CollaborationResponse__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__msg__CollaborationResponse__Sequence__init(dec_control__msg__CollaborationResponse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__CollaborationResponse * data = NULL;

  if (size) {
    data = (dec_control__msg__CollaborationResponse *)allocator.zero_allocate(size, sizeof(dec_control__msg__CollaborationResponse), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__msg__CollaborationResponse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__msg__CollaborationResponse__fini(&data[i - 1]);
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
dec_control__msg__CollaborationResponse__Sequence__fini(dec_control__msg__CollaborationResponse__Sequence * array)
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
      dec_control__msg__CollaborationResponse__fini(&array->data[i]);
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

dec_control__msg__CollaborationResponse__Sequence *
dec_control__msg__CollaborationResponse__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__CollaborationResponse__Sequence * array = (dec_control__msg__CollaborationResponse__Sequence *)allocator.allocate(sizeof(dec_control__msg__CollaborationResponse__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__msg__CollaborationResponse__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__msg__CollaborationResponse__Sequence__destroy(dec_control__msg__CollaborationResponse__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__msg__CollaborationResponse__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__msg__CollaborationResponse__Sequence__are_equal(const dec_control__msg__CollaborationResponse__Sequence * lhs, const dec_control__msg__CollaborationResponse__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__msg__CollaborationResponse__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__msg__CollaborationResponse__Sequence__copy(
  const dec_control__msg__CollaborationResponse__Sequence * input,
  dec_control__msg__CollaborationResponse__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__msg__CollaborationResponse);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__msg__CollaborationResponse * data =
      (dec_control__msg__CollaborationResponse *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__msg__CollaborationResponse__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__msg__CollaborationResponse__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__msg__CollaborationResponse__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
