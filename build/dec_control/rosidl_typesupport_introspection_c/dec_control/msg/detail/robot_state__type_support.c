// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/RobotState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/robot_state__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/robot_state__functions.h"
#include "dec_control/msg/detail/robot_state__struct.h"


// Include directives for member types
// Member `capabilities`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__RobotState__init(message_memory);
}

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_fini_function(void * message_memory)
{
  dec_control__msg__RobotState__fini(message_memory);
}

size_t dec_control__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t dec_control__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__orientation(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__orientation(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__orientation(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__orientation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__orientation(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__orientation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__orientation(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t dec_control__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__capabilities(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__capabilities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__capabilities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__capabilities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__capabilities(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__capabilities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__capabilities(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__RobotState__rosidl_typesupport_introspection_c__resize_function__RobotState__capabilities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_member_array[6] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RobotState, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RobotState, position),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__position,  // size() function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__position,  // get_const(index) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__position,  // get(index) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__position,  // fetch(index, &value) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RobotState, orientation),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__orientation,  // size() function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__orientation,  // get_const(index) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__orientation,  // get(index) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__orientation,  // fetch(index, &value) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__orientation,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "capabilities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RobotState, capabilities),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__capabilities,  // size() function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__capabilities,  // get_const(index) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__capabilities,  // get(index) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__capabilities,  // fetch(index, &value) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__capabilities,  // assign(index, value) function pointer
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__resize_function__RobotState__capabilities  // resize(index) function pointer
  },
  {
    "workload",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RobotState, workload),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RobotState, failed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_members = {
  "dec_control__msg",  // message namespace
  "RobotState",  // message name
  6,  // number of fields
  sizeof(dec_control__msg__RobotState),
  dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_member_array,  // message members
  dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle = {
  0,
  &dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, RobotState)() {
  if (!dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
