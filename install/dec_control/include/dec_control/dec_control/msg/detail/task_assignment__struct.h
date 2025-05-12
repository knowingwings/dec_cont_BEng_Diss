// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/TaskAssignment.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__STRUCT_H_

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
// Member 'robot_ids'
// Member 'prices'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/TaskAssignment in the package dec_control.
/**
  * Current task assignments
 */
typedef struct dec_control__msg__TaskAssignment
{
  rosidl_runtime_c__int32__Sequence task_ids;
  rosidl_runtime_c__int32__Sequence robot_ids;
  rosidl_runtime_c__double__Sequence prices;
} dec_control__msg__TaskAssignment;

// Struct for a sequence of dec_control__msg__TaskAssignment.
typedef struct dec_control__msg__TaskAssignment__Sequence
{
  dec_control__msg__TaskAssignment * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__TaskAssignment__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__STRUCT_H_
