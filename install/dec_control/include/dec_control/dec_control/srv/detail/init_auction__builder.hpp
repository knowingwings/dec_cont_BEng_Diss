// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:srv/InitAuction.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__BUILDER_HPP_
#define DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/srv/detail/init_auction__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace srv
{

namespace builder
{

class Init_InitAuction_Request_capabilities
{
public:
  explicit Init_InitAuction_Request_capabilities(::dec_control::srv::InitAuction_Request & msg)
  : msg_(msg)
  {}
  ::dec_control::srv::InitAuction_Request capabilities(::dec_control::srv::InitAuction_Request::_capabilities_type arg)
  {
    msg_.capabilities = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::srv::InitAuction_Request msg_;
};

class Init_InitAuction_Request_task
{
public:
  Init_InitAuction_Request_task()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InitAuction_Request_capabilities task(::dec_control::srv::InitAuction_Request::_task_type arg)
  {
    msg_.task = std::move(arg);
    return Init_InitAuction_Request_capabilities(msg_);
  }

private:
  ::dec_control::srv::InitAuction_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::srv::InitAuction_Request>()
{
  return dec_control::srv::builder::Init_InitAuction_Request_task();
}

}  // namespace dec_control


namespace dec_control
{

namespace srv
{

namespace builder
{

class Init_InitAuction_Response_success
{
public:
  explicit Init_InitAuction_Response_success(::dec_control::srv::InitAuction_Response & msg)
  : msg_(msg)
  {}
  ::dec_control::srv::InitAuction_Response success(::dec_control::srv::InitAuction_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::srv::InitAuction_Response msg_;
};

class Init_InitAuction_Response_auction_id
{
public:
  Init_InitAuction_Response_auction_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InitAuction_Response_success auction_id(::dec_control::srv::InitAuction_Response::_auction_id_type arg)
  {
    msg_.auction_id = std::move(arg);
    return Init_InitAuction_Response_success(msg_);
  }

private:
  ::dec_control::srv::InitAuction_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::srv::InitAuction_Response>()
{
  return dec_control::srv::builder::Init_InitAuction_Response_auction_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__BUILDER_HPP_
