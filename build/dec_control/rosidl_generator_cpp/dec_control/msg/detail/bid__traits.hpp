// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/Bid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__BID__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__BID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/bid__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Bid & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: bid_value
  {
    out << "bid_value: ";
    rosidl_generator_traits::value_to_yaml(msg.bid_value, out);
    out << ", ";
  }

  // member: utility
  {
    out << "utility: ";
    rosidl_generator_traits::value_to_yaml(msg.utility, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Bid & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: bid_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bid_value: ";
    rosidl_generator_traits::value_to_yaml(msg.bid_value, out);
    out << "\n";
  }

  // member: utility
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "utility: ";
    rosidl_generator_traits::value_to_yaml(msg.utility, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Bid & msg, bool use_flow_style = false)
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
  const dec_control::msg::Bid & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::Bid & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::Bid>()
{
  return "dec_control::msg::Bid";
}

template<>
inline const char * name<dec_control::msg::Bid>()
{
  return "dec_control/msg/Bid";
}

template<>
struct has_fixed_size<dec_control::msg::Bid>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dec_control::msg::Bid>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dec_control::msg::Bid>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__BID__TRAITS_HPP_
