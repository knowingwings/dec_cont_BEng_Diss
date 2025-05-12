// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dec_control:msg/CapabilityUpdate.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dec_control/msg/detail/capability_update__struct.hpp"
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

void CapabilityUpdate_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dec_control::msg::CapabilityUpdate(_init);
}

void CapabilityUpdate_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dec_control::msg::CapabilityUpdate *>(message_memory);
  typed_message->~CapabilityUpdate();
}

size_t size_function__CapabilityUpdate__capabilities(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CapabilityUpdate__capabilities(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CapabilityUpdate__capabilities(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CapabilityUpdate__capabilities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CapabilityUpdate__capabilities(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CapabilityUpdate__capabilities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CapabilityUpdate__capabilities(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CapabilityUpdate__capabilities(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CapabilityUpdate__degradation_mask(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CapabilityUpdate__degradation_mask(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CapabilityUpdate__degradation_mask(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CapabilityUpdate__degradation_mask(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CapabilityUpdate__degradation_mask(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CapabilityUpdate__degradation_mask(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CapabilityUpdate__degradation_mask(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CapabilityUpdate__degradation_mask(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CapabilityUpdate_message_member_array[5] = {
  {
    "robot_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::CapabilityUpdate, robot_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "capabilities",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::CapabilityUpdate, capabilities),  // bytes offset in struct
    nullptr,  // default value
    size_function__CapabilityUpdate__capabilities,  // size() function pointer
    get_const_function__CapabilityUpdate__capabilities,  // get_const(index) function pointer
    get_function__CapabilityUpdate__capabilities,  // get(index) function pointer
    fetch_function__CapabilityUpdate__capabilities,  // fetch(index, &value) function pointer
    assign_function__CapabilityUpdate__capabilities,  // assign(index, value) function pointer
    resize_function__CapabilityUpdate__capabilities  // resize(index) function pointer
  },
  {
    "degradation_mask",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::CapabilityUpdate, degradation_mask),  // bytes offset in struct
    nullptr,  // default value
    size_function__CapabilityUpdate__degradation_mask,  // size() function pointer
    get_const_function__CapabilityUpdate__degradation_mask,  // get_const(index) function pointer
    get_function__CapabilityUpdate__degradation_mask,  // get(index) function pointer
    fetch_function__CapabilityUpdate__degradation_mask,  // fetch(index, &value) function pointer
    assign_function__CapabilityUpdate__degradation_mask,  // assign(index, value) function pointer
    resize_function__CapabilityUpdate__degradation_mask  // resize(index) function pointer
  },
  {
    "in_recovery",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::CapabilityUpdate, in_recovery),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::CapabilityUpdate, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CapabilityUpdate_message_members = {
  "dec_control::msg",  // message namespace
  "CapabilityUpdate",  // message name
  5,  // number of fields
  sizeof(dec_control::msg::CapabilityUpdate),
  CapabilityUpdate_message_member_array,  // message members
  CapabilityUpdate_init_function,  // function to initialize message memory (memory has to be allocated)
  CapabilityUpdate_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CapabilityUpdate_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CapabilityUpdate_message_members,
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
get_message_type_support_handle<dec_control::msg::CapabilityUpdate>()
{
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::CapabilityUpdate_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dec_control, msg, CapabilityUpdate)() {
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::CapabilityUpdate_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
