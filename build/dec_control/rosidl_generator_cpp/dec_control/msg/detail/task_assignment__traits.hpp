// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/TaskAssignment.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/task_assignment__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskAssignment & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_ids
  {
    if (msg.task_ids.size() == 0) {
      out << "task_ids: []";
    } else {
      out << "task_ids: [";
      size_t pending_items = msg.task_ids.size();
      for (auto item : msg.task_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: robot_ids
  {
    if (msg.robot_ids.size() == 0) {
      out << "robot_ids: []";
    } else {
      out << "robot_ids: [";
      size_t pending_items = msg.robot_ids.size();
      for (auto item : msg.robot_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: prices
  {
    if (msg.prices.size() == 0) {
      out << "prices: []";
    } else {
      out << "prices: [";
      size_t pending_items = msg.prices.size();
      for (auto item : msg.prices) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TaskAssignment & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.task_ids.size() == 0) {
      out << "task_ids: []\n";
    } else {
      out << "task_ids:\n";
      for (auto item : msg.task_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: robot_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_ids.size() == 0) {
      out << "robot_ids: []\n";
    } else {
      out << "robot_ids:\n";
      for (auto item : msg.robot_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: prices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.prices.size() == 0) {
      out << "prices: []\n";
    } else {
      out << "prices:\n";
      for (auto item : msg.prices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TaskAssignment & msg, bool use_flow_style = false)
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
  const dec_control::msg::TaskAssignment & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::TaskAssignment & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::TaskAssignment>()
{
  return "dec_control::msg::TaskAssignment";
}

template<>
inline const char * name<dec_control::msg::TaskAssignment>()
{
  return "dec_control/msg/TaskAssignment";
}

template<>
struct has_fixed_size<dec_control::msg::TaskAssignment>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::msg::TaskAssignment>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::msg::TaskAssignment>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__TRAITS_HPP_
