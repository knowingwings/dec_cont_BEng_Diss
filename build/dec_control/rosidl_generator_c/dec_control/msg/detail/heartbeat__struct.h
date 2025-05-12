// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__HEARTBEAT__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__HEARTBEAT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Heartbeat in the package dec_control.
/**
  * Heartbeat message for failure detection
 */
typedef struct dec_control__msg__Heartbeat
{
  int32_t robot_id;
  int64_t timestamp;
  uint8_t status;
} dec_control__msg__Heartbeat;

// Struct for a sequence of dec_control__msg__Heartbeat.
typedef struct dec_control__msg__Heartbeat__Sequence
{
  dec_control__msg__Heartbeat * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__Heartbeat__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__HEARTBEAT__STRUCT_H_
