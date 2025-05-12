// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/TaskAssignment.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/task_assignment__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_TaskAssignment_prices
{
public:
  explicit Init_TaskAssignment_prices(::dec_control::msg::TaskAssignment & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::TaskAssignment prices(::dec_control::msg::TaskAssignment::_prices_type arg)
  {
    msg_.prices = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::TaskAssignment msg_;
};

class Init_TaskAssignment_robot_ids
{
public:
  explicit Init_TaskAssignment_robot_ids(::dec_control::msg::TaskAssignment & msg)
  : msg_(msg)
  {}
  Init_TaskAssignment_prices robot_ids(::dec_control::msg::TaskAssignment::_robot_ids_type arg)
  {
    msg_.robot_ids = std::move(arg);
    return Init_TaskAssignment_prices(msg_);
  }

private:
  ::dec_control::msg::TaskAssignment msg_;
};

class Init_TaskAssignment_task_ids
{
public:
  Init_TaskAssignment_task_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskAssignment_robot_ids task_ids(::dec_control::msg::TaskAssignment::_task_ids_type arg)
  {
    msg_.task_ids = std::move(arg);
    return Init_TaskAssignment_robot_ids(msg_);
  }

private:
  ::dec_control::msg::TaskAssignment msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::TaskAssignment>()
{
  return dec_control::msg::builder::Init_TaskAssignment_task_ids();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__BUILDER_HPP_
