// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'recovery_type'
#include "rosidl_runtime_c/string.h"
// Member 'degraded_capabilities'
// Member 'infeasible_tasks'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RecoveryStatus in the package dec_control.
/**
  * msg/RecoveryStatus.msg
 */
typedef struct dec_control__msg__RecoveryStatus
{
  int32_t robot_id;
  /// "PARTIAL", "COMPLETE", "TASK_INFEASIBLE"
  rosidl_runtime_c__String recovery_type;
  rosidl_runtime_c__float__Sequence degraded_capabilities;
  rosidl_runtime_c__int32__Sequence infeasible_tasks;
  /// milliseconds since epoch
  int64_t timestamp;
} dec_control__msg__RecoveryStatus;

// Struct for a sequence of dec_control__msg__RecoveryStatus.
typedef struct dec_control__msg__RecoveryStatus__Sequence
{
  dec_control__msg__RecoveryStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__RecoveryStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__STRUCT_H_
