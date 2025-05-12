// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dec_control/msg/detail/recovery_status__struct.hpp"
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

void RecoveryStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dec_control::msg::RecoveryStatus(_init);
}

void RecoveryStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dec_control::msg::RecoveryStatus *>(message_memory);
  typed_message->~RecoveryStatus();
}

size_t size_function__RecoveryStatus__degraded_capabilities(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RecoveryStatus__degraded_capabilities(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RecoveryStatus__degraded_capabilities(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RecoveryStatus__degraded_capabilities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RecoveryStatus__degraded_capabilities(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RecoveryStatus__degraded_capabilities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RecoveryStatus__degraded_capabilities(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RecoveryStatus__degraded_capabilities(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RecoveryStatus__infeasible_tasks(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RecoveryStatus__infeasible_tasks(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__RecoveryStatus__infeasible_tasks(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__RecoveryStatus__infeasible_tasks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__RecoveryStatus__infeasible_tasks(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__RecoveryStatus__infeasible_tasks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__RecoveryStatus__infeasible_tasks(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__RecoveryStatus__infeasible_tasks(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RecoveryStatus_message_member_array[5] = {
  {
    "robot_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::RecoveryStatus, robot_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "recovery_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::RecoveryStatus, recovery_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "degraded_capabilities",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::RecoveryStatus, degraded_capabilities),  // bytes offset in struct
    nullptr,  // default value
    size_function__RecoveryStatus__degraded_capabilities,  // size() function pointer
    get_const_function__RecoveryStatus__degraded_capabilities,  // get_const(index) function pointer
    get_function__RecoveryStatus__degraded_capabilities,  // get(index) function pointer
    fetch_function__RecoveryStatus__degraded_capabilities,  // fetch(index, &value) function pointer
    assign_function__RecoveryStatus__degraded_capabilities,  // assign(index, value) function pointer
    resize_function__RecoveryStatus__degraded_capabilities  // resize(index) function pointer
  },
  {
    "infeasible_tasks",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::RecoveryStatus, infeasible_tasks),  // bytes offset in struct
    nullptr,  // default value
    size_function__RecoveryStatus__infeasible_tasks,  // size() function pointer
    get_const_function__RecoveryStatus__infeasible_tasks,  // get_const(index) function pointer
    get_function__RecoveryStatus__infeasible_tasks,  // get(index) function pointer
    fetch_function__RecoveryStatus__infeasible_tasks,  // fetch(index, &value) function pointer
    assign_function__RecoveryStatus__infeasible_tasks,  // assign(index, value) function pointer
    resize_function__RecoveryStatus__infeasible_tasks  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::RecoveryStatus, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RecoveryStatus_message_members = {
  "dec_control::msg",  // message namespace
  "RecoveryStatus",  // message name
  5,  // number of fields
  sizeof(dec_control::msg::RecoveryStatus),
  RecoveryStatus_message_member_array,  // message members
  RecoveryStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  RecoveryStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RecoveryStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RecoveryStatus_message_members,
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
get_message_type_support_handle<dec_control::msg::RecoveryStatus>()
{
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::RecoveryStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dec_control, msg, RecoveryStatus)() {
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::RecoveryStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
