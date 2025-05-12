// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/Bid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__BID__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__BID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Bid in the package dec_control.
/**
  * Bid information
 */
typedef struct dec_control__msg__Bid
{
  int32_t robot_id;
  int32_t task_id;
  double bid_value;
  double utility;
} dec_control__msg__Bid;

// Struct for a sequence of dec_control__msg__Bid.
typedef struct dec_control__msg__Bid__Sequence
{
  dec_control__msg__Bid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__Bid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__BID__STRUCT_H_
