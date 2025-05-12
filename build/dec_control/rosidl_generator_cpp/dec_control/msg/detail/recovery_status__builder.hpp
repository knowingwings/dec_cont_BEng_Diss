// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/recovery_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_RecoveryStatus_timestamp
{
public:
  explicit Init_RecoveryStatus_timestamp(::dec_control::msg::RecoveryStatus & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::RecoveryStatus timestamp(::dec_control::msg::RecoveryStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::RecoveryStatus msg_;
};

class Init_RecoveryStatus_infeasible_tasks
{
public:
  explicit Init_RecoveryStatus_infeasible_tasks(::dec_control::msg::RecoveryStatus & msg)
  : msg_(msg)
  {}
  Init_RecoveryStatus_timestamp infeasible_tasks(::dec_control::msg::RecoveryStatus::_infeasible_tasks_type arg)
  {
    msg_.infeasible_tasks = std::move(arg);
    return Init_RecoveryStatus_timestamp(msg_);
  }

private:
  ::dec_control::msg::RecoveryStatus msg_;
};

class Init_RecoveryStatus_degraded_capabilities
{
public:
  explicit Init_RecoveryStatus_degraded_capabilities(::dec_control::msg::RecoveryStatus & msg)
  : msg_(msg)
  {}
  Init_RecoveryStatus_infeasible_tasks degraded_capabilities(::dec_control::msg::RecoveryStatus::_degraded_capabilities_type arg)
  {
    msg_.degraded_capabilities = std::move(arg);
    return Init_RecoveryStatus_infeasible_tasks(msg_);
  }

private:
  ::dec_control::msg::RecoveryStatus msg_;
};

class Init_RecoveryStatus_recovery_type
{
public:
  explicit Init_RecoveryStatus_recovery_type(::dec_control::msg::RecoveryStatus & msg)
  : msg_(msg)
  {}
  Init_RecoveryStatus_degraded_capabilities recovery_type(::dec_control::msg::RecoveryStatus::_recovery_type_type arg)
  {
    msg_.recovery_type = std::move(arg);
    return Init_RecoveryStatus_degraded_capabilities(msg_);
  }

private:
  ::dec_control::msg::RecoveryStatus msg_;
};

class Init_RecoveryStatus_robot_id
{
public:
  Init_RecoveryStatus_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RecoveryStatus_recovery_type robot_id(::dec_control::msg::RecoveryStatus::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RecoveryStatus_recovery_type(msg_);
  }

private:
  ::dec_control::msg::RecoveryStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::RecoveryStatus>()
{
  return dec_control::msg::builder::Init_RecoveryStatus_robot_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__BUILDER_HPP_
