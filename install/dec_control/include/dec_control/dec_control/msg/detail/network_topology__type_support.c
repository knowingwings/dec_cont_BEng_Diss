// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/network_topology__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/network_topology__functions.h"
#include "dec_control/msg/detail/network_topology__struct.h"


// Include directives for member types
// Member `adjacency_matrix`
// Member `link_quality`
// Member `link_latency`
// Member `link_reliability`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__NetworkTopology__init(message_memory);
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_fini_function(void * message_memory)
{
  dec_control__msg__NetworkTopology__fini(message_memory);
}

size_t dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__adjacency_matrix(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__adjacency_matrix(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__adjacency_matrix(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__adjacency_matrix(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__adjacency_matrix(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__adjacency_matrix(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__adjacency_matrix(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__adjacency_matrix(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__link_quality(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_quality(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_quality(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__link_quality(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_quality(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__link_quality(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_quality(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__link_quality(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__link_latency(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_latency(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_latency(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__link_latency(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_latency(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__link_latency(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_latency(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__link_latency(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__link_reliability(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_reliability(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_reliability(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__link_reliability(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_reliability(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__link_reliability(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_reliability(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__link_reliability(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_member_array[6] = {
  {
    "num_robots",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__NetworkTopology, num_robots),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "adjacency_matrix",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__NetworkTopology, adjacency_matrix),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__adjacency_matrix,  // size() function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__adjacency_matrix,  // get_const(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__adjacency_matrix,  // get(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__adjacency_matrix,  // fetch(index, &value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__adjacency_matrix,  // assign(index, value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__adjacency_matrix  // resize(index) function pointer
  },
  {
    "link_quality",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__NetworkTopology, link_quality),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__link_quality,  // size() function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_quality,  // get_const(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_quality,  // get(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__link_quality,  // fetch(index, &value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__link_quality,  // assign(index, value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__link_quality  // resize(index) function pointer
  },
  {
    "link_latency",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__NetworkTopology, link_latency),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__link_latency,  // size() function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_latency,  // get_const(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_latency,  // get(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__link_latency,  // fetch(index, &value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__link_latency,  // assign(index, value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__link_latency  // resize(index) function pointer
  },
  {
    "link_reliability",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__NetworkTopology, link_reliability),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__size_function__NetworkTopology__link_reliability,  // size() function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_const_function__NetworkTopology__link_reliability,  // get_const(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__get_function__NetworkTopology__link_reliability,  // get(index) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__fetch_function__NetworkTopology__link_reliability,  // fetch(index, &value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__assign_function__NetworkTopology__link_reliability,  // assign(index, value) function pointer
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__resize_function__NetworkTopology__link_reliability  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__NetworkTopology, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_members = {
  "dec_control__msg",  // message namespace
  "NetworkTopology",  // message name
  6,  // number of fields
  sizeof(dec_control__msg__NetworkTopology),
  dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_member_array,  // message members
  dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_type_support_handle = {
  0,
  &dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, NetworkTopology)() {
  if (!dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__NetworkTopology__rosidl_typesupport_introspection_c__NetworkTopology_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
