// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_Task_requires_collaboration
{
public:
  explicit Init_Task_requires_collaboration(::dec_control::msg::Task & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::Task requires_collaboration(::dec_control::msg::Task::_requires_collaboration_type arg)
  {
    msg_.requires_collaboration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::Task msg_;
};

class Init_Task_prerequisites
{
public:
  explicit Init_Task_prerequisites(::dec_control::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_requires_collaboration prerequisites(::dec_control::msg::Task::_prerequisites_type arg)
  {
    msg_.prerequisites = std::move(arg);
    return Init_Task_requires_collaboration(msg_);
  }

private:
  ::dec_control::msg::Task msg_;
};

class Init_Task_execution_time
{
public:
  explicit Init_Task_execution_time(::dec_control::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_prerequisites execution_time(::dec_control::msg::Task::_execution_time_type arg)
  {
    msg_.execution_time = std::move(arg);
    return Init_Task_prerequisites(msg_);
  }

private:
  ::dec_control::msg::Task msg_;
};

class Init_Task_capabilities_required
{
public:
  explicit Init_Task_capabilities_required(::dec_control::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_execution_time capabilities_required(::dec_control::msg::Task::_capabilities_required_type arg)
  {
    msg_.capabilities_required = std::move(arg);
    return Init_Task_execution_time(msg_);
  }

private:
  ::dec_control::msg::Task msg_;
};

class Init_Task_position
{
public:
  explicit Init_Task_position(::dec_control::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_capabilities_required position(::dec_control::msg::Task::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Task_capabilities_required(msg_);
  }

private:
  ::dec_control::msg::Task msg_;
};

class Init_Task_id
{
public:
  Init_Task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Task_position id(::dec_control::msg::Task::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Task_position(msg_);
  }

private:
  ::dec_control::msg::Task msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::Task>()
{
  return dec_control::msg::builder::Init_Task_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK__BUILDER_HPP_
