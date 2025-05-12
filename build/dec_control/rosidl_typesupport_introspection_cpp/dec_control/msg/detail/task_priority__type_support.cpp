// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dec_control/msg/detail/task_priority__struct.hpp"
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

void TaskPriority_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dec_control::msg::TaskPriority(_init);
}

void TaskPriority_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dec_control::msg::TaskPriority *>(message_memory);
  typed_message->~TaskPriority();
}

size_t size_function__TaskPriority__task_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TaskPriority__task_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__TaskPriority__task_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__TaskPriority__task_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__TaskPriority__task_ids(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__TaskPriority__task_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__TaskPriority__task_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__TaskPriority__task_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TaskPriority__priorities(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TaskPriority__priorities(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__TaskPriority__priorities(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__TaskPriority__priorities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TaskPriority__priorities(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TaskPriority__priorities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TaskPriority__priorities(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__TaskPriority__priorities(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TaskPriority__on_critical_path(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__TaskPriority__on_critical_path(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__TaskPriority__on_critical_path(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__TaskPriority__on_critical_path(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TaskPriority__slack_times(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TaskPriority__slack_times(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__TaskPriority__slack_times(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__TaskPriority__slack_times(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TaskPriority__slack_times(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TaskPriority__slack_times(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TaskPriority__slack_times(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__TaskPriority__slack_times(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TaskPriority_message_member_array[4] = {
  {
    "task_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::TaskPriority, task_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__TaskPriority__task_ids,  // size() function pointer
    get_const_function__TaskPriority__task_ids,  // get_const(index) function pointer
    get_function__TaskPriority__task_ids,  // get(index) function pointer
    fetch_function__TaskPriority__task_ids,  // fetch(index, &value) function pointer
    assign_function__TaskPriority__task_ids,  // assign(index, value) function pointer
    resize_function__TaskPriority__task_ids  // resize(index) function pointer
  },
  {
    "priorities",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::TaskPriority, priorities),  // bytes offset in struct
    nullptr,  // default value
    size_function__TaskPriority__priorities,  // size() function pointer
    get_const_function__TaskPriority__priorities,  // get_const(index) function pointer
    get_function__TaskPriority__priorities,  // get(index) function pointer
    fetch_function__TaskPriority__priorities,  // fetch(index, &value) function pointer
    assign_function__TaskPriority__priorities,  // assign(index, value) function pointer
    resize_function__TaskPriority__priorities  // resize(index) function pointer
  },
  {
    "on_critical_path",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::TaskPriority, on_critical_path),  // bytes offset in struct
    nullptr,  // default value
    size_function__TaskPriority__on_critical_path,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__TaskPriority__on_critical_path,  // fetch(index, &value) function pointer
    assign_function__TaskPriority__on_critical_path,  // assign(index, value) function pointer
    resize_function__TaskPriority__on_critical_path  // resize(index) function pointer
  },
  {
    "slack_times",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::TaskPriority, slack_times),  // bytes offset in struct
    nullptr,  // default value
    size_function__TaskPriority__slack_times,  // size() function pointer
    get_const_function__TaskPriority__slack_times,  // get_const(index) function pointer
    get_function__TaskPriority__slack_times,  // get(index) function pointer
    fetch_function__TaskPriority__slack_times,  // fetch(index, &value) function pointer
    assign_function__TaskPriority__slack_times,  // assign(index, value) function pointer
    resize_function__TaskPriority__slack_times  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TaskPriority_message_members = {
  "dec_control::msg",  // message namespace
  "TaskPriority",  // message name
  4,  // number of fields
  sizeof(dec_control::msg::TaskPriority),
  TaskPriority_message_member_array,  // message members
  TaskPriority_init_function,  // function to initialize message memory (memory has to be allocated)
  TaskPriority_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TaskPriority_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TaskPriority_message_members,
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
get_message_type_support_handle<dec_control::msg::TaskPriority>()
{
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::TaskPriority_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dec_control, msg, TaskPriority)() {
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::TaskPriority_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
