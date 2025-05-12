// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/task_priority__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_TaskPriority_slack_times
{
public:
  explicit Init_TaskPriority_slack_times(::dec_control::msg::TaskPriority & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::TaskPriority slack_times(::dec_control::msg::TaskPriority::_slack_times_type arg)
  {
    msg_.slack_times = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::TaskPriority msg_;
};

class Init_TaskPriority_on_critical_path
{
public:
  explicit Init_TaskPriority_on_critical_path(::dec_control::msg::TaskPriority & msg)
  : msg_(msg)
  {}
  Init_TaskPriority_slack_times on_critical_path(::dec_control::msg::TaskPriority::_on_critical_path_type arg)
  {
    msg_.on_critical_path = std::move(arg);
    return Init_TaskPriority_slack_times(msg_);
  }

private:
  ::dec_control::msg::TaskPriority msg_;
};

class Init_TaskPriority_priorities
{
public:
  explicit Init_TaskPriority_priorities(::dec_control::msg::TaskPriority & msg)
  : msg_(msg)
  {}
  Init_TaskPriority_on_critical_path priorities(::dec_control::msg::TaskPriority::_priorities_type arg)
  {
    msg_.priorities = std::move(arg);
    return Init_TaskPriority_on_critical_path(msg_);
  }

private:
  ::dec_control::msg::TaskPriority msg_;
};

class Init_TaskPriority_task_ids
{
public:
  Init_TaskPriority_task_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskPriority_priorities task_ids(::dec_control::msg::TaskPriority::_task_ids_type arg)
  {
    msg_.task_ids = std::move(arg);
    return Init_TaskPriority_priorities(msg_);
  }

private:
  ::dec_control::msg::TaskPriority msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::TaskPriority>()
{
  return dec_control::msg::builder::Init_TaskPriority_task_ids();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__BUILDER_HPP_
