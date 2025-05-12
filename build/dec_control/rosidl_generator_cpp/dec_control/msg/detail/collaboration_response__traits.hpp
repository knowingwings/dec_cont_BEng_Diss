// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/CollaborationResponse.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__COLLABORATION_RESPONSE__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__COLLABORATION_RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/collaboration_response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const CollaborationResponse & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: agree_to_role
  {
    out << "agree_to_role: ";
    rosidl_generator_traits::value_to_yaml(msg.agree_to_role, out);
    out << ", ";
  }

  // member: is_leader
  {
    out << "is_leader: ";
    rosidl_generator_traits::value_to_yaml(msg.is_leader, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CollaborationResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: agree_to_role
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "agree_to_role: ";
    rosidl_generator_traits::value_to_yaml(msg.agree_to_role, out);
    out << "\n";
  }

  // member: is_leader
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_leader: ";
    rosidl_generator_traits::value_to_yaml(msg.is_leader, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CollaborationResponse & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dec_control

namespace rosidl_generator_traits
{

[[deprecated("use dec_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dec_control::msg::CollaborationResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::CollaborationResponse & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::CollaborationResponse>()
{
  return "dec_control::msg::CollaborationResponse";
}

template<>
inline const char * name<dec_control::msg::CollaborationResponse>()
{
  return "dec_control/msg/CollaborationResponse";
}

template<>
struct has_fixed_size<dec_control::msg::CollaborationResponse>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dec_control::msg::CollaborationResponse>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dec_control::msg::CollaborationResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__COLLABORATION_RESPONSE__TRAITS_HPP_
