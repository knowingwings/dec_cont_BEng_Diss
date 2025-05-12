// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:srv/SubmitBid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__BUILDER_HPP_
#define DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/srv/detail/submit_bid__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace srv
{

namespace builder
{

class Init_SubmitBid_Request_resource_availability
{
public:
  explicit Init_SubmitBid_Request_resource_availability(::dec_control::srv::SubmitBid_Request & msg)
  : msg_(msg)
  {}
  ::dec_control::srv::SubmitBid_Request resource_availability(::dec_control::srv::SubmitBid_Request::_resource_availability_type arg)
  {
    msg_.resource_availability = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::srv::SubmitBid_Request msg_;
};

class Init_SubmitBid_Request_bid_value
{
public:
  explicit Init_SubmitBid_Request_bid_value(::dec_control::srv::SubmitBid_Request & msg)
  : msg_(msg)
  {}
  Init_SubmitBid_Request_resource_availability bid_value(::dec_control::srv::SubmitBid_Request::_bid_value_type arg)
  {
    msg_.bid_value = std::move(arg);
    return Init_SubmitBid_Request_resource_availability(msg_);
  }

private:
  ::dec_control::srv::SubmitBid_Request msg_;
};

class Init_SubmitBid_Request_robot_id
{
public:
  explicit Init_SubmitBid_Request_robot_id(::dec_control::srv::SubmitBid_Request & msg)
  : msg_(msg)
  {}
  Init_SubmitBid_Request_bid_value robot_id(::dec_control::srv::SubmitBid_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_SubmitBid_Request_bid_value(msg_);
  }

private:
  ::dec_control::srv::SubmitBid_Request msg_;
};

class Init_SubmitBid_Request_auction_id
{
public:
  Init_SubmitBid_Request_auction_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SubmitBid_Request_robot_id auction_id(::dec_control::srv::SubmitBid_Request::_auction_id_type arg)
  {
    msg_.auction_id = std::move(arg);
    return Init_SubmitBid_Request_robot_id(msg_);
  }

private:
  ::dec_control::srv::SubmitBid_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::srv::SubmitBid_Request>()
{
  return dec_control::srv::builder::Init_SubmitBid_Request_auction_id();
}

}  // namespace dec_control


namespace dec_control
{

namespace srv
{

namespace builder
{

class Init_SubmitBid_Response_accepted
{
public:
  Init_SubmitBid_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dec_control::srv::SubmitBid_Response accepted(::dec_control::srv::SubmitBid_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::srv::SubmitBid_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::srv::SubmitBid_Response>()
{
  return dec_control::srv::builder::Init_SubmitBid_Response_accepted();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__BUILDER_HPP_
