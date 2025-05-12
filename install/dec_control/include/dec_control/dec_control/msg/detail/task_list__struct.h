// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/TaskList.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_LIST__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__TASK_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tasks'
#include "dec_control/msg/detail/task__struct.h"

/// Struct defined in msg/TaskList in the package dec_control.
/**
  * List of tasks
 */
typedef struct dec_control__msg__TaskList
{
  dec_control__msg__Task__Sequence tasks;
} dec_control__msg__TaskList;

// Struct for a sequence of dec_control__msg__TaskList.
typedef struct dec_control__msg__TaskList__Sequence
{
  dec_control__msg__TaskList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__TaskList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_LIST__STRUCT_H_
