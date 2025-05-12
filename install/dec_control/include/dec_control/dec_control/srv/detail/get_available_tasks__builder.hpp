// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:srv/GetAvailableTasks.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__BUILDER_HPP_
#define DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/srv/detail/get_available_tasks__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace srv
{

namespace builder
{

class Init_GetAvailableTasks_Request_robot_id
{
public:
  Init_GetAvailableTasks_Request_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dec_control::srv::GetAvailableTasks_Request robot_id(::dec_control::srv::GetAvailableTasks_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::srv::GetAvailableTasks_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::srv::GetAvailableTasks_Request>()
{
  return dec_control::srv::builder::Init_GetAvailableTasks_Request_robot_id();
}

}  // namespace dec_control


namespace dec_control
{

namespace srv
{

namespace builder
{

class Init_GetAvailableTasks_Response_available_tasks
{
public:
  Init_GetAvailableTasks_Response_available_tasks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dec_control::srv::GetAvailableTasks_Response available_tasks(::dec_control::srv::GetAvailableTasks_Response::_available_tasks_type arg)
  {
    msg_.available_tasks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::srv::GetAvailableTasks_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::srv::GetAvailableTasks_Response>()
{
  return dec_control::srv::builder::Init_GetAvailableTasks_Response_available_tasks();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__BUILDER_HPP_
