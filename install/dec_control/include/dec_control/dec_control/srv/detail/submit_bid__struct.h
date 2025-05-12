// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:srv/SubmitBid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__STRUCT_H_
#define DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'auction_id'
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"
// Member 'resource_availability'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/SubmitBid in the package dec_control.
typedef struct dec_control__srv__SubmitBid_Request
{
  rosidl_runtime_c__String auction_id;
  rosidl_runtime_c__String robot_id;
  float bid_value;
  rosidl_runtime_c__float__Sequence resource_availability;
} dec_control__srv__SubmitBid_Request;

// Struct for a sequence of dec_control__srv__SubmitBid_Request.
typedef struct dec_control__srv__SubmitBid_Request__Sequence
{
  dec_control__srv__SubmitBid_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__srv__SubmitBid_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SubmitBid in the package dec_control.
typedef struct dec_control__srv__SubmitBid_Response
{
  bool accepted;
} dec_control__srv__SubmitBid_Response;

// Struct for a sequence of dec_control__srv__SubmitBid_Response.
typedef struct dec_control__srv__SubmitBid_Response__Sequence
{
  dec_control__srv__SubmitBid_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__srv__SubmitBid_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__STRUCT_H_
