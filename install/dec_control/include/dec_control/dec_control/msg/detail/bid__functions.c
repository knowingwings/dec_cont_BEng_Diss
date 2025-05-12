// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:msg/Bid.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/bid__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
dec_control__msg__Bid__init(dec_control__msg__Bid * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // task_id
  // bid_value
  // utility
  return true;
}

void
dec_control__msg__Bid__fini(dec_control__msg__Bid * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // task_id
  // bid_value
  // utility
}

bool
dec_control__msg__Bid__are_equal(const dec_control__msg__Bid * lhs, const dec_control__msg__Bid * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // task_id
  if (lhs->task_id != rhs->task_id) {
    return false;
  }
  // bid_value
  if (lhs->bid_value != rhs->bid_value) {
    return false;
  }
  // utility
  if (lhs->utility != rhs->utility) {
    return false;
  }
  return true;
}

bool
dec_control__msg__Bid__copy(
  const dec_control__msg__Bid * input,
  dec_control__msg__Bid * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // task_id
  output->task_id = input->task_id;
  // bid_value
  output->bid_value = input->bid_value;
  // utility
  output->utility = input->utility;
  return true;
}

dec_control__msg__Bid *
dec_control__msg__Bid__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__Bid * msg = (dec_control__msg__Bid *)allocator.allocate(sizeof(dec_control__msg__Bid), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__msg__Bid));
  bool success = dec_control__msg__Bid__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__msg__Bid__destroy(dec_control__msg__Bid * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__msg__Bid__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__msg__Bid__Sequence__init(dec_control__msg__Bid__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__Bid * data = NULL;

  if (size) {
    data = (dec_control__msg__Bid *)allocator.zero_allocate(size, sizeof(dec_control__msg__Bid), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__msg__Bid__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__msg__Bid__fini(&data[i - 1]);
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
dec_control__msg__Bid__Sequence__fini(dec_control__msg__Bid__Sequence * array)
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
      dec_control__msg__Bid__fini(&array->data[i]);
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

dec_control__msg__Bid__Sequence *
dec_control__msg__Bid__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__Bid__Sequence * array = (dec_control__msg__Bid__Sequence *)allocator.allocate(sizeof(dec_control__msg__Bid__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__msg__Bid__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__msg__Bid__Sequence__destroy(dec_control__msg__Bid__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__msg__Bid__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__msg__Bid__Sequence__are_equal(const dec_control__msg__Bid__Sequence * lhs, const dec_control__msg__Bid__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__msg__Bid__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__msg__Bid__Sequence__copy(
  const dec_control__msg__Bid__Sequence * input,
  dec_control__msg__Bid__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__msg__Bid);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__msg__Bid * data =
      (dec_control__msg__Bid *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__msg__Bid__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__msg__Bid__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__msg__Bid__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
