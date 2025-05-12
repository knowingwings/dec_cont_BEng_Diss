// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_Heartbeat_status
{
public:
  explicit Init_Heartbeat_status(::dec_control::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::Heartbeat status(::dec_control::msg::Heartbeat::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::Heartbeat msg_;
};

class Init_Heartbeat_timestamp
{
public:
  explicit Init_Heartbeat_timestamp(::dec_control::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  Init_Heartbeat_status timestamp(::dec_control::msg::Heartbeat::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Heartbeat_status(msg_);
  }

private:
  ::dec_control::msg::Heartbeat msg_;
};

class Init_Heartbeat_robot_id
{
public:
  Init_Heartbeat_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Heartbeat_timestamp robot_id(::dec_control::msg::Heartbeat::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_Heartbeat_timestamp(msg_);
  }

private:
  ::dec_control::msg::Heartbeat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::Heartbeat>()
{
  return dec_control::msg::builder::Init_Heartbeat_robot_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_
