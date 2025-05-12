// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/Bid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__BID__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__BID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/bid__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_Bid_utility
{
public:
  explicit Init_Bid_utility(::dec_control::msg::Bid & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::Bid utility(::dec_control::msg::Bid::_utility_type arg)
  {
    msg_.utility = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::Bid msg_;
};

class Init_Bid_bid_value
{
public:
  explicit Init_Bid_bid_value(::dec_control::msg::Bid & msg)
  : msg_(msg)
  {}
  Init_Bid_utility bid_value(::dec_control::msg::Bid::_bid_value_type arg)
  {
    msg_.bid_value = std::move(arg);
    return Init_Bid_utility(msg_);
  }

private:
  ::dec_control::msg::Bid msg_;
};

class Init_Bid_task_id
{
public:
  explicit Init_Bid_task_id(::dec_control::msg::Bid & msg)
  : msg_(msg)
  {}
  Init_Bid_bid_value task_id(::dec_control::msg::Bid::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_Bid_bid_value(msg_);
  }

private:
  ::dec_control::msg::Bid msg_;
};

class Init_Bid_robot_id
{
public:
  Init_Bid_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Bid_task_id robot_id(::dec_control::msg::Bid::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_Bid_task_id(msg_);
  }

private:
  ::dec_control::msg::Bid msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::Bid>()
{
  return dec_control::msg::builder::Init_Bid_robot_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__BID__BUILDER_HPP_
