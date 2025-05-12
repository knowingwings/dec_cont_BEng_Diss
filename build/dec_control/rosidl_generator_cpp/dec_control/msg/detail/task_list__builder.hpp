// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/TaskList.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_LIST__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/task_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_TaskList_tasks
{
public:
  Init_TaskList_tasks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dec_control::msg::TaskList tasks(::dec_control::msg::TaskList::_tasks_type arg)
  {
    msg_.tasks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::TaskList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::TaskList>()
{
  return dec_control::msg::builder::Init_TaskList_tasks();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_LIST__BUILDER_HPP_
