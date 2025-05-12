// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dec_control:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dec_control/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dec_control/msg/detail/robot_state__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace dec_control
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dec_control
cdr_serialize(
  const dec_control::msg::RobotState & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dec_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dec_control::msg::RobotState & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dec_control
get_serialized_size(
  const dec_control::msg::RobotState & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dec_control
max_serialized_size_RobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dec_control

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dec_control
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dec_control, msg, RobotState)();

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
