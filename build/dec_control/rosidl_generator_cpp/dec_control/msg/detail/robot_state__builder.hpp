// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_RobotState_failed
{
public:
  explicit Init_RobotState_failed(::dec_control::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::RobotState failed(::dec_control::msg::RobotState::_failed_type arg)
  {
    msg_.failed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::RobotState msg_;
};

class Init_RobotState_workload
{
public:
  explicit Init_RobotState_workload(::dec_control::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_failed workload(::dec_control::msg::RobotState::_workload_type arg)
  {
    msg_.workload = std::move(arg);
    return Init_RobotState_failed(msg_);
  }

private:
  ::dec_control::msg::RobotState msg_;
};

class Init_RobotState_capabilities
{
public:
  explicit Init_RobotState_capabilities(::dec_control::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_workload capabilities(::dec_control::msg::RobotState::_capabilities_type arg)
  {
    msg_.capabilities = std::move(arg);
    return Init_RobotState_workload(msg_);
  }

private:
  ::dec_control::msg::RobotState msg_;
};

class Init_RobotState_orientation
{
public:
  explicit Init_RobotState_orientation(::dec_control::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_capabilities orientation(::dec_control::msg::RobotState::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_RobotState_capabilities(msg_);
  }

private:
  ::dec_control::msg::RobotState msg_;
};

class Init_RobotState_position
{
public:
  explicit Init_RobotState_position(::dec_control::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_orientation position(::dec_control::msg::RobotState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_RobotState_orientation(msg_);
  }

private:
  ::dec_control::msg::RobotState msg_;
};

class Init_RobotState_id
{
public:
  Init_RobotState_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_position id(::dec_control::msg::RobotState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_RobotState_position(msg_);
  }

private:
  ::dec_control::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::RobotState>()
{
  return dec_control::msg::builder::Init_RobotState_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
