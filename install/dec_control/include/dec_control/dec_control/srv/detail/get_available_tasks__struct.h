// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:srv/GetAvailableTasks.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__STRUCT_H_
#define DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetAvailableTasks in the package dec_control.
typedef struct dec_control__srv__GetAvailableTasks_Request
{
  rosidl_runtime_c__String robot_id;
} dec_control__srv__GetAvailableTasks_Request;

// Struct for a sequence of dec_control__srv__GetAvailableTasks_Request.
typedef struct dec_control__srv__GetAvailableTasks_Request__Sequence
{
  dec_control__srv__GetAvailableTasks_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__srv__GetAvailableTasks_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'available_tasks'
#include "dec_control/msg/detail/task__struct.h"

/// Struct defined in srv/GetAvailableTasks in the package dec_control.
typedef struct dec_control__srv__GetAvailableTasks_Response
{
  dec_control__msg__Task__Sequence available_tasks;
} dec_control__srv__GetAvailableTasks_Response;

// Struct for a sequence of dec_control__srv__GetAvailableTasks_Response.
typedef struct dec_control__srv__GetAvailableTasks_Response__Sequence
{
  dec_control__srv__GetAvailableTasks_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__srv__GetAvailableTasks_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__STRUCT_H_
