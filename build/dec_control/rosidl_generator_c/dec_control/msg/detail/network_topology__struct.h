// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'adjacency_matrix'
// Member 'link_quality'
// Member 'link_latency'
// Member 'link_reliability'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/NetworkTopology in the package dec_control.
/**
  * msg/NetworkTopology.msg
 */
typedef struct dec_control__msg__NetworkTopology
{
  int32_t num_robots;
  rosidl_runtime_c__float__Sequence adjacency_matrix;
  /// Quality metric for each link (0-1)
  rosidl_runtime_c__float__Sequence link_quality;
  /// Communication latency for each link in seconds
  rosidl_runtime_c__float__Sequence link_latency;
  /// Reliability metric for each link (0-1)
  rosidl_runtime_c__float__Sequence link_reliability;
  /// Timestamp in milliseconds
  int64_t timestamp;
} dec_control__msg__NetworkTopology;

// Struct for a sequence of dec_control__msg__NetworkTopology.
typedef struct dec_control__msg__NetworkTopology__Sequence
{
  dec_control__msg__NetworkTopology * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__NetworkTopology__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__STRUCT_H_
