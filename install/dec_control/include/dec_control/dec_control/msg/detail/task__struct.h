// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'capabilities_required'
// Member 'prerequisites'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Task in the package dec_control.
/**
  * Task representation
 */
typedef struct dec_control__msg__Task
{
  int32_t id;
  /// 3D position in world frame
  float position[3];
  /// Required capabilities
  rosidl_runtime_c__float__Sequence capabilities_required;
  /// Estimated completion time
  float execution_time;
  /// IDs of prerequisite tasks
  rosidl_runtime_c__int32__Sequence prerequisites;
  /// Whether this task requires both robots
  bool requires_collaboration;
} dec_control__msg__Task;

// Struct for a sequence of dec_control__msg__Task.
typedef struct dec_control__msg__Task__Sequence
{
  dec_control__msg__Task * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__Task__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__TASK__STRUCT_H_
