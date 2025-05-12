// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/SynchronizationSignal.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__SYNCHRONIZATION_SIGNAL__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__SYNCHRONIZATION_SIGNAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SynchronizationSignal in the package dec_control.
/**
  * msg/SynchronizationSignal.msg
 */
typedef struct dec_control__msg__SynchronizationSignal
{
  int32_t task_id;
  int32_t robot_id;
  int32_t sync_point;
  /// READY, ACKNOWLEDGE, COMPLETED
  rosidl_runtime_c__String status;
  /// milliseconds since epoch
  int64_t timestamp;
} dec_control__msg__SynchronizationSignal;

// Struct for a sequence of dec_control__msg__SynchronizationSignal.
typedef struct dec_control__msg__SynchronizationSignal__Sequence
{
  dec_control__msg__SynchronizationSignal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__SynchronizationSignal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__SYNCHRONIZATION_SIGNAL__STRUCT_H_
