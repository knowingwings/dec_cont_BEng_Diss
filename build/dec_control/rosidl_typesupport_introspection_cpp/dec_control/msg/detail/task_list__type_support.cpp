// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dec_control:msg/TaskList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dec_control/msg/detail/task_list__struct.hpp"
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

void TaskList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dec_control::msg::TaskList(_init);
}

void TaskList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dec_control::msg::TaskList *>(message_memory);
  typed_message->~TaskList();
}

size_t size_function__TaskList__tasks(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dec_control::msg::Task> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TaskList__tasks(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dec_control::msg::Task> *>(untyped_member);
  return &member[index];
}

void * get_function__TaskList__tasks(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dec_control::msg::Task> *>(untyped_member);
  return &member[index];
}

void fetch_function__TaskList__tasks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dec_control::msg::Task *>(
    get_const_function__TaskList__tasks(untyped_member, index));
  auto & value = *reinterpret_cast<dec_control::msg::Task *>(untyped_value);
  value = item;
}

void assign_function__TaskList__tasks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dec_control::msg::Task *>(
    get_function__TaskList__tasks(untyped_member, index));
  const auto & value = *reinterpret_cast<const dec_control::msg::Task *>(untyped_value);
  item = value;
}

void resize_function__TaskList__tasks(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dec_control::msg::Task> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TaskList_message_member_array[1] = {
  {
    "tasks",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dec_control::msg::Task>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control::msg::TaskList, tasks),  // bytes offset in struct
    nullptr,  // default value
    size_function__TaskList__tasks,  // size() function pointer
    get_const_function__TaskList__tasks,  // get_const(index) function pointer
    get_function__TaskList__tasks,  // get(index) function pointer
    fetch_function__TaskList__tasks,  // fetch(index, &value) function pointer
    assign_function__TaskList__tasks,  // assign(index, value) function pointer
    resize_function__TaskList__tasks  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TaskList_message_members = {
  "dec_control::msg",  // message namespace
  "TaskList",  // message name
  1,  // number of fields
  sizeof(dec_control::msg::TaskList),
  TaskList_message_member_array,  // message members
  TaskList_init_function,  // function to initialize message memory (memory has to be allocated)
  TaskList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TaskList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TaskList_message_members,
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
get_message_type_support_handle<dec_control::msg::TaskList>()
{
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::TaskList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dec_control, msg, TaskList)() {
  return &::dec_control::msg::rosidl_typesupport_introspection_cpp::TaskList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
