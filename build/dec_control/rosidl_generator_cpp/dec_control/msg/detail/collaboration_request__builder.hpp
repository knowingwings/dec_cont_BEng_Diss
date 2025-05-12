// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/CollaborationRequest.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/collaboration_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_CollaborationRequest_timestamp
{
public:
  explicit Init_CollaborationRequest_timestamp(::dec_control::msg::CollaborationRequest & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::CollaborationRequest timestamp(::dec_control::msg::CollaborationRequest::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::CollaborationRequest msg_;
};

class Init_CollaborationRequest_leadership_score
{
public:
  explicit Init_CollaborationRequest_leadership_score(::dec_control::msg::CollaborationRequest & msg)
  : msg_(msg)
  {}
  Init_CollaborationRequest_timestamp leadership_score(::dec_control::msg::CollaborationRequest::_leadership_score_type arg)
  {
    msg_.leadership_score = std::move(arg);
    return Init_CollaborationRequest_timestamp(msg_);
  }

private:
  ::dec_control::msg::CollaborationRequest msg_;
};

class Init_CollaborationRequest_robot_id
{
public:
  explicit Init_CollaborationRequest_robot_id(::dec_control::msg::CollaborationRequest & msg)
  : msg_(msg)
  {}
  Init_CollaborationRequest_leadership_score robot_id(::dec_control::msg::CollaborationRequest::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_CollaborationRequest_leadership_score(msg_);
  }

private:
  ::dec_control::msg::CollaborationRequest msg_;
};

class Init_CollaborationRequest_task_id
{
public:
  Init_CollaborationRequest_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CollaborationRequest_robot_id task_id(::dec_control::msg::CollaborationRequest::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_CollaborationRequest_robot_id(msg_);
  }

private:
  ::dec_control::msg::CollaborationRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::CollaborationRequest>()
{
  return dec_control::msg::builder::Init_CollaborationRequest_task_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__BUILDER_HPP_
