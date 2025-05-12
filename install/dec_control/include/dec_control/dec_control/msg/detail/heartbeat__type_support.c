// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/heartbeat__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/heartbeat__functions.h"
#include "dec_control/msg/detail/heartbeat__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__Heartbeat__init(message_memory);
}

void dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_fini_function(void * message_memory)
{
  dec_control__msg__Heartbeat__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_member_array[3] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Heartbeat, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Heartbeat, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Heartbeat, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_members = {
  "dec_control__msg",  // message namespace
  "Heartbeat",  // message name
  3,  // number of fields
  sizeof(dec_control__msg__Heartbeat),
  dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_member_array,  // message members
  dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_type_support_handle = {
  0,
  &dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, Heartbeat)() {
  if (!dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__Heartbeat__rosidl_typesupport_introspection_c__Heartbeat_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
