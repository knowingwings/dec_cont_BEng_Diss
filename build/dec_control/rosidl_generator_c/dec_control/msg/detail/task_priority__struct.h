// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task_ids'
// Member 'priorities'
// Member 'on_critical_path'
// Member 'slack_times'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/TaskPriority in the package dec_control.
/**
  * msg/TaskPriority.msg
 */
typedef struct dec_control__msg__TaskPriority
{
  rosidl_runtime_c__int32__Sequence task_ids;
  rosidl_runtime_c__double__Sequence priorities;
  rosidl_runtime_c__boolean__Sequence on_critical_path;
  rosidl_runtime_c__double__Sequence slack_times;
} dec_control__msg__TaskPriority;

// Struct for a sequence of dec_control__msg__TaskPriority.
typedef struct dec_control__msg__TaskPriority__Sequence
{
  dec_control__msg__TaskPriority * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__TaskPriority__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__STRUCT_H_
