// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/task_priority__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/task_priority__functions.h"
#include "dec_control/msg/detail/task_priority__struct.h"


// Include directives for member types
// Member `task_ids`
// Member `priorities`
// Member `on_critical_path`
// Member `slack_times`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__TaskPriority__init(message_memory);
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_fini_function(void * message_memory)
{
  dec_control__msg__TaskPriority__fini(message_memory);
}

size_t dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__task_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__task_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__task_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__task_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__task_ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__task_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__task_ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__task_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__priorities(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__priorities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__priorities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__priorities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__priorities(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__priorities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__priorities(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__priorities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__on_critical_path(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__on_critical_path(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__on_critical_path(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__on_critical_path(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__on_critical_path(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__on_critical_path(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__on_critical_path(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__on_critical_path(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__slack_times(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__slack_times(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__slack_times(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__slack_times(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__slack_times(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__slack_times(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__slack_times(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__slack_times(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_member_array[4] = {
  {
    "task_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskPriority, task_ids),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__task_ids,  // size() function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__task_ids,  // get_const(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__task_ids,  // get(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__task_ids,  // fetch(index, &value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__task_ids,  // assign(index, value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__task_ids  // resize(index) function pointer
  },
  {
    "priorities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskPriority, priorities),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__priorities,  // size() function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__priorities,  // get_const(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__priorities,  // get(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__priorities,  // fetch(index, &value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__priorities,  // assign(index, value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__priorities  // resize(index) function pointer
  },
  {
    "on_critical_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskPriority, on_critical_path),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__on_critical_path,  // size() function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__on_critical_path,  // get_const(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__on_critical_path,  // get(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__on_critical_path,  // fetch(index, &value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__on_critical_path,  // assign(index, value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__on_critical_path  // resize(index) function pointer
  },
  {
    "slack_times",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskPriority, slack_times),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__size_function__TaskPriority__slack_times,  // size() function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_const_function__TaskPriority__slack_times,  // get_const(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__get_function__TaskPriority__slack_times,  // get(index) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__fetch_function__TaskPriority__slack_times,  // fetch(index, &value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__assign_function__TaskPriority__slack_times,  // assign(index, value) function pointer
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__resize_function__TaskPriority__slack_times  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_members = {
  "dec_control__msg",  // message namespace
  "TaskPriority",  // message name
  4,  // number of fields
  sizeof(dec_control__msg__TaskPriority),
  dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_member_array,  // message members
  dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_type_support_handle = {
  0,
  &dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, TaskPriority)() {
  if (!dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__TaskPriority__rosidl_typesupport_introspection_c__TaskPriority_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
