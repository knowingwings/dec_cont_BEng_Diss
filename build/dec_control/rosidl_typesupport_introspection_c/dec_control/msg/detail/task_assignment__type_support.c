// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/TaskAssignment.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/task_assignment__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/task_assignment__functions.h"
#include "dec_control/msg/detail/task_assignment__struct.h"


// Include directives for member types
// Member `task_ids`
// Member `robot_ids`
// Member `prices`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__TaskAssignment__init(message_memory);
}

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_fini_function(void * message_memory)
{
  dec_control__msg__TaskAssignment__fini(message_memory);
}

size_t dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__size_function__TaskAssignment__task_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__task_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__task_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__fetch_function__TaskAssignment__task_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__task_ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__assign_function__TaskAssignment__task_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__task_ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__resize_function__TaskAssignment__task_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__size_function__TaskAssignment__robot_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__robot_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__robot_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__fetch_function__TaskAssignment__robot_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__robot_ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__assign_function__TaskAssignment__robot_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__robot_ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__resize_function__TaskAssignment__robot_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__size_function__TaskAssignment__prices(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__prices(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__prices(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__fetch_function__TaskAssignment__prices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__prices(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__assign_function__TaskAssignment__prices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__prices(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__resize_function__TaskAssignment__prices(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_member_array[3] = {
  {
    "task_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskAssignment, task_ids),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__size_function__TaskAssignment__task_ids,  // size() function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__task_ids,  // get_const(index) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__task_ids,  // get(index) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__fetch_function__TaskAssignment__task_ids,  // fetch(index, &value) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__assign_function__TaskAssignment__task_ids,  // assign(index, value) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__resize_function__TaskAssignment__task_ids  // resize(index) function pointer
  },
  {
    "robot_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskAssignment, robot_ids),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__size_function__TaskAssignment__robot_ids,  // size() function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__robot_ids,  // get_const(index) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__robot_ids,  // get(index) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__fetch_function__TaskAssignment__robot_ids,  // fetch(index, &value) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__assign_function__TaskAssignment__robot_ids,  // assign(index, value) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__resize_function__TaskAssignment__robot_ids  // resize(index) function pointer
  },
  {
    "prices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskAssignment, prices),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__size_function__TaskAssignment__prices,  // size() function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_const_function__TaskAssignment__prices,  // get_const(index) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__get_function__TaskAssignment__prices,  // get(index) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__fetch_function__TaskAssignment__prices,  // fetch(index, &value) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__assign_function__TaskAssignment__prices,  // assign(index, value) function pointer
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__resize_function__TaskAssignment__prices  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_members = {
  "dec_control__msg",  // message namespace
  "TaskAssignment",  // message name
  3,  // number of fields
  sizeof(dec_control__msg__TaskAssignment),
  dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_member_array,  // message members
  dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_type_support_handle = {
  0,
  &dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, TaskAssignment)() {
  if (!dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__TaskAssignment__rosidl_typesupport_introspection_c__TaskAssignment_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
