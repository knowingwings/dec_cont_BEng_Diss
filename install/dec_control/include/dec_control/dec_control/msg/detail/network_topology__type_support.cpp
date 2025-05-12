// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dec_control/msg/detail/network_topology__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dec_control
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void NetworkTopology_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dec_control::msg::NetworkTopology(_init);
}

void NetworkTopology_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dec_control::msg::NetworkTopology *>(message_memory);
  typed_message->~NetworkTopology();
}

size_t size_function__NetworkTopology__adjacency_matrix(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NetworkTopology__adjacency_matrix(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__NetworkTopology__adjacency_matrix(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__NetworkTopology__adjacency_matrix(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__NetworkTopology__adjacency_matrix(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__NetworkTopology__adjacency_matrix(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__NetworkTopology__adjacency_matrix(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__NetworkTopology__adjacency_matrix(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__NetworkTopology__link_quality(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NetworkTopology__link_quality(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__NetworkTopology__link_quality(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__NetworkTopology__link_quality(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__NetworkTopology__link_quality(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__NetworkTopology__link_quality(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__NetworkTopology__link_quality(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__NetworkTopology__link_quality(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__NetworkTopology__link_latency(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NetworkTopology__link_latency(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__NetworkTopology__link_latency(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__NetworkTopology__link_latency(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__NetworkTopology__link_latency(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__NetworkTopology__link_latency(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__NetworkTopology__link_latency(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__NetworkTopology__link_latency(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__NetworkTopology__link_reliability(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NetworkTopology__link_reliability(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__NetworkTopology__link_reliability(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__NetworkTopology__link_reliability(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__NetworkTopology__link_reliability(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__NetworkTopology__link_reliability(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__NetworkTopology__link_reliability(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__NetworkTopology__link_reliability(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NetworkTopology_message_member_array[6] = {
  {
    "num_robots",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::NetworkTopology, num_robots),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "adjacency_matrix",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::NetworkTopology, adjacency_matrix),  // bytes offset in struct
    nullptr,  // default value
    size_function__NetworkTopology__adjacency_matrix,  // size() function pointer
    get_const_function__NetworkTopology__adjacency_matrix,  // get_const(index) function pointer
    get_function__NetworkTopology__adjacency_matrix,  // get(index) function pointer
    fetch_function__NetworkTopology__adjacency_matrix,  // fetch(index, &value) function pointer
    assign_function__NetworkTopology__adjacency_matrix,  // assign(index, value) function pointer
    resize_function__NetworkTopology__adjacency_matrix  // resize(index) function pointer
  },
  {
    "link_quality",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::NetworkTopology, link_quality),  // bytes offset in struct
    nullptr,  // default value
    size_function__NetworkTopology__link_quality,  // size() function pointer
    get_const_function__NetworkTopology__link_quality,  // get_const(index) function pointer
    get_function__NetworkTopology__link_quality,  // get(index) function pointer
    fetch_function__NetworkTopology__link_quality,  // fetch(index, &value) function pointer
    assign_function__NetworkTopology__link_quality,  // assign(index, value) function pointer
    resize_function__NetworkTopology__link_quality  // resize(index) function pointer
  },
  {
    "link_latency",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::NetworkTopology, link_latency),  // bytes offset in struct
    nullptr,  // default value
    size_function__NetworkTopology__link_latency,  // size() function pointer
    get_const_function__NetworkTopology__link_latency,  // get_const(index) function pointer
    get_function__NetworkTopology__link_latency,  // get(index) function pointer
    fetch_function__NetworkTopology__link_latency,  // fetch(index, &value) function pointer
    assign_function__NetworkTopology__link_latency,  // assign(index, value) function pointer
    resize_function__NetworkTopology__link_latency  // resize(index) function pointer
  },
  {
    "link_reliability",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::NetworkTopology, link_reliability),  // bytes offset in struct
    nullptr,  // default value
    size_function__NetworkTopology__link_reliability,  // size() function pointer
    get_const_function__NetworkTopology__link_reliability,  // get_const(index) function pointer
    get_function__NetworkTopology__link_reliability,  // get(index) function pointer
    fetch_function__NetworkTopology__link_reliability,  // fetch(index, &value) function pointer
    assign_function__NetworkTopology__link_reliability,  // assign(index, value) function pointer
    resize_function__NetworkTopology__link_reliability  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::NetworkTopology, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NetworkTopology_message_members = {
  "dec_control::msg",  // message namespace
  "NetworkTopology",  // message name
  6,  // number of fields
  sizeof(dec_control::msg::NetworkTopology),
  NetworkTopology_message_member_array,  // message members
  NetworkTopology_init_function,  // function to initialize message memory (memory has to be allocated)
  NetworkTopology_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NetworkTopology_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NetworkTopology_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dec_control


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dec_control::msg::NetworkTopology>()
{
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::NetworkTopology_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dec_control, msg, NetworkTopology)() {
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::NetworkTopology_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
