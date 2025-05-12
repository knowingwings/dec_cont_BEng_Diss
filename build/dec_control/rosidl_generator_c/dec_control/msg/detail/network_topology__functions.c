// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/network_topology__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `adjacency_matrix`
// Member `link_quality`
// Member `link_latency`
// Member `link_reliability`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dec_control__msg__NetworkTopology__init(dec_control__msg__NetworkTopology * msg)
{
  if (!msg) {
    return false;
  }
  // num_robots
  // adjacency_matrix
  if (!rosidl_runtime_c__float__Sequence__init(&msg->adjacency_matrix, 0)) {
    dec_control__msg__NetworkTopology__fini(msg);
    return false;
  }
  // link_quality
  if (!rosidl_runtime_c__float__Sequence__init(&msg->link_quality, 0)) {
    dec_control__msg__NetworkTopology__fini(msg);
    return false;
  }
  // link_latency
  if (!rosidl_runtime_c__float__Sequence__init(&msg->link_latency, 0)) {
    dec_control__msg__NetworkTopology__fini(msg);
    return false;
  }
  // link_reliability
  if (!rosidl_runtime_c__float__Sequence__init(&msg->link_reliability, 0)) {
    dec_control__msg__NetworkTopology__fini(msg);
    return false;
  }
  // timestamp
  return true;
}

void
dec_control__msg__NetworkTopology__fini(dec_control__msg__NetworkTopology * msg)
{
  if (!msg) {
    return;
  }
  // num_robots
  // adjacency_matrix
  rosidl_runtime_c__float__Sequence__fini(&msg->adjacency_matrix);
  // link_quality
  rosidl_runtime_c__float__Sequence__fini(&msg->link_quality);
  // link_latency
  rosidl_runtime_c__float__Sequence__fini(&msg->link_latency);
  // link_reliability
  rosidl_runtime_c__float__Sequence__fini(&msg->link_reliability);
  // timestamp
}

bool
dec_control__msg__NetworkTopology__are_equal(const dec_control__msg__NetworkTopology * lhs, const dec_control__msg__NetworkTopology * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num_robots
  if (lhs->num_robots != rhs->num_robots) {
    return false;
  }
  // adjacency_matrix
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->adjacency_matrix), &(rhs->adjacency_matrix)))
  {
    return false;
  }
  // link_quality
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->link_quality), &(rhs->link_quality)))
  {
    return false;
  }
  // link_latency
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->link_latency), &(rhs->link_latency)))
  {
    return false;
  }
  // link_reliability
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->link_reliability), &(rhs->link_reliability)))
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
dec_control__msg__NetworkTopology__copy(
  const dec_control__msg__NetworkTopology * input,
  dec_control__msg__NetworkTopology * output)
{
  if (!input || !output) {
    return false;
  }
  // num_robots
  output->num_robots = input->num_robots;
  // adjacency_matrix
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->adjacency_matrix), &(output->adjacency_matrix)))
  {
    return false;
  }
  // link_quality
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->link_quality), &(output->link_quality)))
  {
    return false;
  }
  // link_latency
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->link_latency), &(output->link_latency)))
  {
    return false;
  }
  // link_reliability
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->link_reliability), &(output->link_reliability)))
  {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  return true;
}

dec_control__msg__NetworkTopology *
dec_control__msg__NetworkTopology__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__NetworkTopology * msg = (dec_control__msg__NetworkTopology *)allocator.allocate(sizeof(dec_control__msg__NetworkTopology), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__msg__NetworkTopology));
  bool success = dec_control__msg__NetworkTopology__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__msg__NetworkTopology__destroy(dec_control__msg__NetworkTopology * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__msg__NetworkTopology__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__msg__NetworkTopology__Sequence__init(dec_control__msg__NetworkTopology__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__NetworkTopology * data = NULL;

  if (size) {
    data = (dec_control__msg__NetworkTopology *)allocator.zero_allocate(size, sizeof(dec_control__msg__NetworkTopology), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__msg__NetworkTopology__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__msg__NetworkTopology__fini(&data[i - 1]);
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
dec_control__msg__NetworkTopology__Sequence__fini(dec_control__msg__NetworkTopology__Sequence * array)
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
      dec_control__msg__NetworkTopology__fini(&array->data[i]);
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

dec_control__msg__NetworkTopology__Sequence *
dec_control__msg__NetworkTopology__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__msg__NetworkTopology__Sequence * array = (dec_control__msg__NetworkTopology__Sequence *)allocator.allocate(sizeof(dec_control__msg__NetworkTopology__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__msg__NetworkTopology__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__msg__NetworkTopology__Sequence__destroy(dec_control__msg__NetworkTopology__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__msg__NetworkTopology__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__msg__NetworkTopology__Sequence__are_equal(const dec_control__msg__NetworkTopology__Sequence * lhs, const dec_control__msg__NetworkTopology__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__msg__NetworkTopology__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__msg__NetworkTopology__Sequence__copy(
  const dec_control__msg__NetworkTopology__Sequence * input,
  dec_control__msg__NetworkTopology__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__msg__NetworkTopology);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__msg__NetworkTopology * data =
      (dec_control__msg__NetworkTopology *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__msg__NetworkTopology__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__msg__NetworkTopology__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__msg__NetworkTopology__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
