// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:msg/CollaborationRequest.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__STRUCT_H_
#define DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CollaborationRequest in the package dec_control.
/**
  * msg/CollaborationRequest.msg
 */
typedef struct dec_control__msg__CollaborationRequest
{
  int32_t task_id;
  int32_t robot_id;
  double leadership_score;
  /// milliseconds since epoch
  int64_t timestamp;
} dec_control__msg__CollaborationRequest;

// Struct for a sequence of dec_control__msg__CollaborationRequest.
typedef struct dec_control__msg__CollaborationRequest__Sequence
{
  dec_control__msg__CollaborationRequest * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__msg__CollaborationRequest__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__STRUCT_H_
