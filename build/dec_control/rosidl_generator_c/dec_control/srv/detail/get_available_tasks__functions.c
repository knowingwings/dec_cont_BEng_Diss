// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dec_control:srv/GetAvailableTasks.idl
// generated code does not contain a copyright notice
#include "dec_control/srv/detail/get_available_tasks__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"

bool
dec_control__srv__GetAvailableTasks_Request__init(dec_control__srv__GetAvailableTasks_Request * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    dec_control__srv__GetAvailableTasks_Request__fini(msg);
    return false;
  }
  return true;
}

void
dec_control__srv__GetAvailableTasks_Request__fini(dec_control__srv__GetAvailableTasks_Request * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
}

bool
dec_control__srv__GetAvailableTasks_Request__are_equal(const dec_control__srv__GetAvailableTasks_Request * lhs, const dec_control__srv__GetAvailableTasks_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  return true;
}

bool
dec_control__srv__GetAvailableTasks_Request__copy(
  const dec_control__srv__GetAvailableTasks_Request * input,
  dec_control__srv__GetAvailableTasks_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  return true;
}

dec_control__srv__GetAvailableTasks_Request *
dec_control__srv__GetAvailableTasks_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__srv__GetAvailableTasks_Request * msg = (dec_control__srv__GetAvailableTasks_Request *)allocator.allocate(sizeof(dec_control__srv__GetAvailableTasks_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__srv__GetAvailableTasks_Request));
  bool success = dec_control__srv__GetAvailableTasks_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__srv__GetAvailableTasks_Request__destroy(dec_control__srv__GetAvailableTasks_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__srv__GetAvailableTasks_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__srv__GetAvailableTasks_Request__Sequence__init(dec_control__srv__GetAvailableTasks_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__srv__GetAvailableTasks_Request * data = NULL;

  if (size) {
    data = (dec_control__srv__GetAvailableTasks_Request *)allocator.zero_allocate(size, sizeof(dec_control__srv__GetAvailableTasks_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__srv__GetAvailableTasks_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__srv__GetAvailableTasks_Request__fini(&data[i - 1]);
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
dec_control__srv__GetAvailableTasks_Request__Sequence__fini(dec_control__srv__GetAvailableTasks_Request__Sequence * array)
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
      dec_control__srv__GetAvailableTasks_Request__fini(&array->data[i]);
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

dec_control__srv__GetAvailableTasks_Request__Sequence *
dec_control__srv__GetAvailableTasks_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__srv__GetAvailableTasks_Request__Sequence * array = (dec_control__srv__GetAvailableTasks_Request__Sequence *)allocator.allocate(sizeof(dec_control__srv__GetAvailableTasks_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__srv__GetAvailableTasks_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__srv__GetAvailableTasks_Request__Sequence__destroy(dec_control__srv__GetAvailableTasks_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__srv__GetAvailableTasks_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__srv__GetAvailableTasks_Request__Sequence__are_equal(const dec_control__srv__GetAvailableTasks_Request__Sequence * lhs, const dec_control__srv__GetAvailableTasks_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__srv__GetAvailableTasks_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__srv__GetAvailableTasks_Request__Sequence__copy(
  const dec_control__srv__GetAvailableTasks_Request__Sequence * input,
  dec_control__srv__GetAvailableTasks_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__srv__GetAvailableTasks_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__srv__GetAvailableTasks_Request * data =
      (dec_control__srv__GetAvailableTasks_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__srv__GetAvailableTasks_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__srv__GetAvailableTasks_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__srv__GetAvailableTasks_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `available_tasks`
#include "dec_control/msg/detail/task__functions.h"

bool
dec_control__srv__GetAvailableTasks_Response__init(dec_control__srv__GetAvailableTasks_Response * msg)
{
  if (!msg) {
    return false;
  }
  // available_tasks
  if (!dec_control__msg__Task__Sequence__init(&msg->available_tasks, 0)) {
    dec_control__srv__GetAvailableTasks_Response__fini(msg);
    return false;
  }
  return true;
}

void
dec_control__srv__GetAvailableTasks_Response__fini(dec_control__srv__GetAvailableTasks_Response * msg)
{
  if (!msg) {
    return;
  }
  // available_tasks
  dec_control__msg__Task__Sequence__fini(&msg->available_tasks);
}

bool
dec_control__srv__GetAvailableTasks_Response__are_equal(const dec_control__srv__GetAvailableTasks_Response * lhs, const dec_control__srv__GetAvailableTasks_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // available_tasks
  if (!dec_control__msg__Task__Sequence__are_equal(
      &(lhs->available_tasks), &(rhs->available_tasks)))
  {
    return false;
  }
  return true;
}

bool
dec_control__srv__GetAvailableTasks_Response__copy(
  const dec_control__srv__GetAvailableTasks_Response * input,
  dec_control__srv__GetAvailableTasks_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // available_tasks
  if (!dec_control__msg__Task__Sequence__copy(
      &(input->available_tasks), &(output->available_tasks)))
  {
    return false;
  }
  return true;
}

dec_control__srv__GetAvailableTasks_Response *
dec_control__srv__GetAvailableTasks_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__srv__GetAvailableTasks_Response * msg = (dec_control__srv__GetAvailableTasks_Response *)allocator.allocate(sizeof(dec_control__srv__GetAvailableTasks_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dec_control__srv__GetAvailableTasks_Response));
  bool success = dec_control__srv__GetAvailableTasks_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dec_control__srv__GetAvailableTasks_Response__destroy(dec_control__srv__GetAvailableTasks_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dec_control__srv__GetAvailableTasks_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dec_control__srv__GetAvailableTasks_Response__Sequence__init(dec_control__srv__GetAvailableTasks_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__srv__GetAvailableTasks_Response * data = NULL;

  if (size) {
    data = (dec_control__srv__GetAvailableTasks_Response *)allocator.zero_allocate(size, sizeof(dec_control__srv__GetAvailableTasks_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dec_control__srv__GetAvailableTasks_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dec_control__srv__GetAvailableTasks_Response__fini(&data[i - 1]);
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
dec_control__srv__GetAvailableTasks_Response__Sequence__fini(dec_control__srv__GetAvailableTasks_Response__Sequence * array)
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
      dec_control__srv__GetAvailableTasks_Response__fini(&array->data[i]);
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

dec_control__srv__GetAvailableTasks_Response__Sequence *
dec_control__srv__GetAvailableTasks_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dec_control__srv__GetAvailableTasks_Response__Sequence * array = (dec_control__srv__GetAvailableTasks_Response__Sequence *)allocator.allocate(sizeof(dec_control__srv__GetAvailableTasks_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dec_control__srv__GetAvailableTasks_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dec_control__srv__GetAvailableTasks_Response__Sequence__destroy(dec_control__srv__GetAvailableTasks_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dec_control__srv__GetAvailableTasks_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dec_control__srv__GetAvailableTasks_Response__Sequence__are_equal(const dec_control__srv__GetAvailableTasks_Response__Sequence * lhs, const dec_control__srv__GetAvailableTasks_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dec_control__srv__GetAvailableTasks_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dec_control__srv__GetAvailableTasks_Response__Sequence__copy(
  const dec_control__srv__GetAvailableTasks_Response__Sequence * input,
  dec_control__srv__GetAvailableTasks_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dec_control__srv__GetAvailableTasks_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dec_control__srv__GetAvailableTasks_Response * data =
      (dec_control__srv__GetAvailableTasks_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dec_control__srv__GetAvailableTasks_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dec_control__srv__GetAvailableTasks_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dec_control__srv__GetAvailableTasks_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
