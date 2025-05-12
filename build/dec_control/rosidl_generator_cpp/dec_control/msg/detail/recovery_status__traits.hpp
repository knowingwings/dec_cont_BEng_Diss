// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/recovery_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const RecoveryStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: recovery_type
  {
    out << "recovery_type: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_type, out);
    out << ", ";
  }

  // member: degraded_capabilities
  {
    if (msg.degraded_capabilities.size() == 0) {
      out << "degraded_capabilities: []";
    } else {
      out << "degraded_capabilities: [";
      size_t pending_items = msg.degraded_capabilities.size();
      for (auto item : msg.degraded_capabilities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: infeasible_tasks
  {
    if (msg.infeasible_tasks.size() == 0) {
      out << "infeasible_tasks: []";
    } else {
      out << "infeasible_tasks: [";
      size_t pending_items = msg.infeasible_tasks.size();
      for (auto item : msg.infeasible_tasks) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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
  const RecoveryStatus & msg,
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

  // member: recovery_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_type: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_type, out);
    out << "\n";
  }

  // member: degraded_capabilities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.degraded_capabilities.size() == 0) {
      out << "degraded_capabilities: []\n";
    } else {
      out << "degraded_capabilities:\n";
      for (auto item : msg.degraded_capabilities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: infeasible_tasks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.infeasible_tasks.size() == 0) {
      out << "infeasible_tasks: []\n";
    } else {
      out << "infeasible_tasks:\n";
      for (auto item : msg.infeasible_tasks) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

inline std::string to_yaml(const RecoveryStatus & msg, bool use_flow_style = false)
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
  const dec_control::msg::RecoveryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::RecoveryStatus & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::RecoveryStatus>()
{
  return "dec_control::msg::RecoveryStatus";
}

template<>
inline const char * name<dec_control::msg::RecoveryStatus>()
{
  return "dec_control/msg/RecoveryStatus";
}

template<>
struct has_fixed_size<dec_control::msg::RecoveryStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::msg::RecoveryStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::msg::RecoveryStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__TRAITS_HPP_
