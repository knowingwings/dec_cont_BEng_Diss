// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/SynchronizationSignal.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__SYNCHRONIZATION_SIGNAL__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__SYNCHRONIZATION_SIGNAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/synchronization_signal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_SynchronizationSignal_timestamp
{
public:
  explicit Init_SynchronizationSignal_timestamp(::dec_control::msg::SynchronizationSignal & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::SynchronizationSignal timestamp(::dec_control::msg::SynchronizationSignal::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::SynchronizationSignal msg_;
};

class Init_SynchronizationSignal_status
{
public:
  explicit Init_SynchronizationSignal_status(::dec_control::msg::SynchronizationSignal & msg)
  : msg_(msg)
  {}
  Init_SynchronizationSignal_timestamp status(::dec_control::msg::SynchronizationSignal::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SynchronizationSignal_timestamp(msg_);
  }

private:
  ::dec_control::msg::SynchronizationSignal msg_;
};

class Init_SynchronizationSignal_sync_point
{
public:
  explicit Init_SynchronizationSignal_sync_point(::dec_control::msg::SynchronizationSignal & msg)
  : msg_(msg)
  {}
  Init_SynchronizationSignal_status sync_point(::dec_control::msg::SynchronizationSignal::_sync_point_type arg)
  {
    msg_.sync_point = std::move(arg);
    return Init_SynchronizationSignal_status(msg_);
  }

private:
  ::dec_control::msg::SynchronizationSignal msg_;
};

class Init_SynchronizationSignal_robot_id
{
public:
  explicit Init_SynchronizationSignal_robot_id(::dec_control::msg::SynchronizationSignal & msg)
  : msg_(msg)
  {}
  Init_SynchronizationSignal_sync_point robot_id(::dec_control::msg::SynchronizationSignal::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_SynchronizationSignal_sync_point(msg_);
  }

private:
  ::dec_control::msg::SynchronizationSignal msg_;
};

class Init_SynchronizationSignal_task_id
{
public:
  Init_SynchronizationSignal_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SynchronizationSignal_robot_id task_id(::dec_control::msg::SynchronizationSignal::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_SynchronizationSignal_robot_id(msg_);
  }

private:
  ::dec_control::msg::SynchronizationSignal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::SynchronizationSignal>()
{
  return dec_control::msg::builder::Init_SynchronizationSignal_task_id();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__SYNCHRONIZATION_SIGNAL__BUILDER_HPP_
