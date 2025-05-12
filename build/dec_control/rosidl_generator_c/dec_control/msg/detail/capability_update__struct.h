// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/CapabilityUpdate.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__STRUCT_H_

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
// Member 'degradation_mask'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/CapabilityUpdate in the package dec_control.
/**
  * msg/CapabilityUpdate.msg
 */
typedef struct dec_control__msg__CapabilityUpdate
{
  int32_t robot_id;
  rosidl_runtime_c__float__Sequence capabilities;
  /// 1 for degraded capability, 0 otherwise
  rosidl_runtime_c__float__Sequence degradation_mask;
  bool in_recovery;
  /// milliseconds since epoch
  int64_t timestamp;
} dec_control__msg__CapabilityUpdate;

// Struct for a sequence of dec_control__msg__CapabilityUpdate.
typedef struct dec_control__msg__CapabilityUpdate__Sequence
{
  dec_control__msg__CapabilityUpdate * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__CapabilityUpdate__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__STRUCT_H_
