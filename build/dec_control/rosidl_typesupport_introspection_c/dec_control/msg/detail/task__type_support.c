// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/Task.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/task__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/task__functions.h"
#include "dec_control/msg/detail/task__struct.h"


// Include directives for member types
// Member `capabilities_required`
// Member `prerequisites`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__Task__init(message_memory);
}

void dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_fini_function(void * message_memory)
{
  dec_control__msg__Task__fini(message_memory);
}

size_t dec_control__msg__Task__rosidl_typesupport_introspection_c__size_function__Task__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void dec_control__msg__Task__rosidl_typesupport_introspection_c__fetch_function__Task__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__Task__rosidl_typesupport_introspection_c__assign_function__Task__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t dec_control__msg__Task__rosidl_typesupport_introspection_c__size_function__Task__capabilities_required(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__capabilities_required(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__capabilities_required(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__Task__rosidl_typesupport_introspection_c__fetch_function__Task__capabilities_required(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__capabilities_required(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__Task__rosidl_typesupport_introspection_c__assign_function__Task__capabilities_required(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__capabilities_required(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__Task__rosidl_typesupport_introspection_c__resize_function__Task__capabilities_required(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t dec_control__msg__Task__rosidl_typesupport_introspection_c__size_function__Task__prerequisites(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__prerequisites(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__prerequisites(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__Task__rosidl_typesupport_introspection_c__fetch_function__Task__prerequisites(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__prerequisites(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void dec_control__msg__Task__rosidl_typesupport_introspection_c__assign_function__Task__prerequisites(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__prerequisites(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__Task__rosidl_typesupport_introspection_c__resize_function__Task__prerequisites(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_member_array[6] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Task, id),  // bytes offset in struct
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
    offsetof(dec_control__msg__Task, position),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__Task__rosidl_typesupport_introspection_c__size_function__Task__position,  // size() function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__position,  // get_const(index) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__position,  // get(index) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__fetch_function__Task__position,  // fetch(index, &value) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__assign_function__Task__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "capabilities_required",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Task, capabilities_required),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__Task__rosidl_typesupport_introspection_c__size_function__Task__capabilities_required,  // size() function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__capabilities_required,  // get_const(index) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__capabilities_required,  // get(index) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__fetch_function__Task__capabilities_required,  // fetch(index, &value) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__assign_function__Task__capabilities_required,  // assign(index, value) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__resize_function__Task__capabilities_required  // resize(index) function pointer
  },
  {
    "execution_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Task, execution_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prerequisites",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Task, prerequisites),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__Task__rosidl_typesupport_introspection_c__size_function__Task__prerequisites,  // size() function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_const_function__Task__prerequisites,  // get_const(index) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__get_function__Task__prerequisites,  // get(index) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__fetch_function__Task__prerequisites,  // fetch(index, &value) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__assign_function__Task__prerequisites,  // assign(index, value) function pointer
    dec_control__msg__Task__rosidl_typesupport_introspection_c__resize_function__Task__prerequisites  // resize(index) function pointer
  },
  {
    "requires_collaboration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__Task, requires_collaboration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_members = {
  "dec_control__msg",  // message namespace
  "Task",  // message name
  6,  // number of fields
  sizeof(dec_control__msg__Task),
  dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_member_array,  // message members
  dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle = {
  0,
  &dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, Task)() {
  if (!dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
