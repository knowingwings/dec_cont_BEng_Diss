// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/CollaborationResponse.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__COLLABORATION_RESPONSE__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__COLLABORATION_RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/collaboration_response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_CollaborationResponse_timestamp
{
public:
  explicit Init_CollaborationResponse_timestamp(::dec_control::msg::CollaborationResponse & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::CollaborationResponse timestamp(::dec_control::msg::CollaborationResponse::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::CollaborationResponse msg_;
};

class Init_CollaborationResponse_is_leader
{
public:
  explicit Init_CollaborationResponse_is_leader(::dec_control::msg::CollaborationResponse & msg)
  : msg_(msg)
  {}
  Init_CollaborationResponse_timestamp is_leader(::dec_control::msg::CollaborationResponse::_is_leader_type arg)
  {
    msg_.is_leader = std::move(arg);
    return Init_CollaborationResponse_timestamp(msg_);
  }

private:
  ::dec_control::msg::CollaborationResponse msg_;
};

class Init_CollaborationResponse_agree_to_role
{
public:
  explicit Init_CollaborationResponse_agree_to_role(::dec_control::msg::CollaborationResponse & msg)
  : msg_(msg)
  {}
  Init_CollaborationResponse_is_leader agree_to_role(::dec_control::msg::CollaborationResponse::_agree_to_role_type arg)
  {
    msg_.agree_to_role = std::move(arg);
    return Init_CollaborationResponse_is_leader(msg_);
  }

private:
  ::dec_control::msg::CollaborationResponse msg_;
};

class Init_CollaborationResponse_robot_id
{
public:
  explicit Init_CollaborationResponse_robot_id(::dec_control::msg::CollaborationResponse & msg)
  : msg_(msg)
  {}
  Init_CollaborationResponse_agree_to_role robot_id(::dec_control::msg::CollaborationResponse::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_CollaborationResponse_agree_to_role(msg_);
  }

private:
  ::dec_control::msg::CollaborationResponse msg_;
};

class Init_CollaborationResponse_task_id
{
public:
  Init_CollaborationResponse_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CollaborationResponse_robot_id task_id(::dec_control::msg::CollaborationResponse::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_CollaborationResponse_robot_id(msg_);
  }

private:
  ::dec_control::msg::CollaborationResponse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::CollaborationResponse>()
{
  return dec_control::msg::builder::Init_CollaborationResponse_task_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__COLLABORATION_RESPONSE__BUILDER_HPP_
