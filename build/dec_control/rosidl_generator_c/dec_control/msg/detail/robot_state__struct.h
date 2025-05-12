// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'capabilities'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RobotState in the package dec_control.
/**
  * Robot state information
 */
typedef struct dec_control__msg__RobotState
{
  int32_t id;
  float position[3];
  /// Roll, pitch, yaw
  float orientation[3];
  rosidl_runtime_c__float__Sequence capabilities;
  double workload;
  bool failed;
} dec_control__msg__RobotState;

// Struct for a sequence of dec_control__msg__RobotState.
typedef struct dec_control__msg__RobotState__Sequence
{
  dec_control__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
