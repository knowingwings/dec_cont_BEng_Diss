// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dec_control:srv/InitAuction.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__STRUCT_H_
#define DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task'
#include "dec_control/msg/detail/task__struct.h"
// Member 'capabilities'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/InitAuction in the package dec_control.
typedef struct dec_control__srv__InitAuction_Request
{
  dec_control__msg__Task task;
  rosidl_runtime_c__float__Sequence capabilities;
} dec_control__srv__InitAuction_Request;

// Struct for a sequence of dec_control__srv__InitAuction_Request.
typedef struct dec_control__srv__InitAuction_Request__Sequence
{
  dec_control__srv__InitAuction_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__srv__InitAuction_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'auction_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/InitAuction in the package dec_control.
typedef struct dec_control__srv__InitAuction_Response
{
  rosidl_runtime_c__String auction_id;
  bool success;
} dec_control__srv__InitAuction_Response;

// Struct for a sequence of dec_control__srv__InitAuction_Response.
typedef struct dec_control__srv__InitAuction_Response__Sequence
{
  dec_control__srv__InitAuction_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dec_control__srv__InitAuction_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__STRUCT_H_
