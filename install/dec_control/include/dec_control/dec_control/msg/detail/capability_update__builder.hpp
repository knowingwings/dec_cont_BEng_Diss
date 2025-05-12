// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/CapabilityUpdate.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/capability_update__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_CapabilityUpdate_timestamp
{
public:
  explicit Init_CapabilityUpdate_timestamp(::dec_control::msg::CapabilityUpdate & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::CapabilityUpdate timestamp(::dec_control::msg::CapabilityUpdate::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::CapabilityUpdate msg_;
};

class Init_CapabilityUpdate_in_recovery
{
public:
  explicit Init_CapabilityUpdate_in_recovery(::dec_control::msg::CapabilityUpdate & msg)
  : msg_(msg)
  {}
  Init_CapabilityUpdate_timestamp in_recovery(::dec_control::msg::CapabilityUpdate::_in_recovery_type arg)
  {
    msg_.in_recovery = std::move(arg);
    return Init_CapabilityUpdate_timestamp(msg_);
  }

private:
  ::dec_control::msg::CapabilityUpdate msg_;
};

class Init_CapabilityUpdate_degradation_mask
{
public:
  explicit Init_CapabilityUpdate_degradation_mask(::dec_control::msg::CapabilityUpdate & msg)
  : msg_(msg)
  {}
  Init_CapabilityUpdate_in_recovery degradation_mask(::dec_control::msg::CapabilityUpdate::_degradation_mask_type arg)
  {
    msg_.degradation_mask = std::move(arg);
    return Init_CapabilityUpdate_in_recovery(msg_);
  }

private:
  ::dec_control::msg::CapabilityUpdate msg_;
};

class Init_CapabilityUpdate_capabilities
{
public:
  explicit Init_CapabilityUpdate_capabilities(::dec_control::msg::CapabilityUpdate & msg)
  : msg_(msg)
  {}
  Init_CapabilityUpdate_degradation_mask capabilities(::dec_control::msg::CapabilityUpdate::_capabilities_type arg)
  {
    msg_.capabilities = std::move(arg);
    return Init_CapabilityUpdate_degradation_mask(msg_);
  }

private:
  ::dec_control::msg::CapabilityUpdate msg_;
};

class Init_CapabilityUpdate_robot_id
{
public:
  Init_CapabilityUpdate_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CapabilityUpdate_capabilities robot_id(::dec_control::msg::CapabilityUpdate::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_CapabilityUpdate_capabilities(msg_);
  }

private:
  ::dec_control::msg::CapabilityUpdate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::CapabilityUpdate>()
{
  return dec_control::msg::builder::Init_CapabilityUpdate_robot_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__BUILDER_HPP_
