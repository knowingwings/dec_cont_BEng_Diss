// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/recovery_status__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/recovery_status__functions.h"
#include "dec_control/msg/detail/recovery_status__struct.h"


// Include directives for member types
// Member `recovery_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `degraded_capabilities`
// Member `infeasible_tasks`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__RecoveryStatus__init(message_memory);
}

void dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_fini_function(void * message_memory)
{
  dec_control__msg__RecoveryStatus__fini(message_memory);
}

size_t dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__size_function__RecoveryStatus__degraded_capabilities(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_const_function__RecoveryStatus__degraded_capabilities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_function__RecoveryStatus__degraded_capabilities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__fetch_function__RecoveryStatus__degraded_capabilities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_const_function__RecoveryStatus__degraded_capabilities(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__assign_function__RecoveryStatus__degraded_capabilities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_function__RecoveryStatus__degraded_capabilities(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__resize_function__RecoveryStatus__degraded_capabilities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__size_function__RecoveryStatus__infeasible_tasks(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_const_function__RecoveryStatus__infeasible_tasks(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_function__RecoveryStatus__infeasible_tasks(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__fetch_function__RecoveryStatus__infeasible_tasks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_const_function__RecoveryStatus__infeasible_tasks(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__assign_function__RecoveryStatus__infeasible_tasks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_function__RecoveryStatus__infeasible_tasks(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__resize_function__RecoveryStatus__infeasible_tasks(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_member_array[5] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RecoveryStatus, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recovery_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RecoveryStatus, recovery_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "degraded_capabilities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RecoveryStatus, degraded_capabilities),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__size_function__RecoveryStatus__degraded_capabilities,  // size() function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_const_function__RecoveryStatus__degraded_capabilities,  // get_const(index) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_function__RecoveryStatus__degraded_capabilities,  // get(index) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__fetch_function__RecoveryStatus__degraded_capabilities,  // fetch(index, &value) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__assign_function__RecoveryStatus__degraded_capabilities,  // assign(index, value) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__resize_function__RecoveryStatus__degraded_capabilities  // resize(index) function pointer
  },
  {
    "infeasible_tasks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RecoveryStatus, infeasible_tasks),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__size_function__RecoveryStatus__infeasible_tasks,  // size() function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_const_function__RecoveryStatus__infeasible_tasks,  // get_const(index) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__get_function__RecoveryStatus__infeasible_tasks,  // get(index) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__fetch_function__RecoveryStatus__infeasible_tasks,  // fetch(index, &value) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__assign_function__RecoveryStatus__infeasible_tasks,  // assign(index, value) function pointer
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__resize_function__RecoveryStatus__infeasible_tasks  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__RecoveryStatus, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_members = {
  "dec_control__msg",  // message namespace
  "RecoveryStatus",  // message name
  5,  // number of fields
  sizeof(dec_control__msg__RecoveryStatus),
  dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_member_array,  // message members
  dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_type_support_handle = {
  0,
  &dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, RecoveryStatus)() {
  if (!dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__RecoveryStatus__rosidl_typesupport_introspection_c__RecoveryStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
