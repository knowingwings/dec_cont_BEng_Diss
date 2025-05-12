// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: orientation
  {
    if (msg.orientation.size() == 0) {
      out << "orientation: []";
    } else {
      out << "orientation: [";
      size_t pending_items = msg.orientation.size();
      for (auto item : msg.orientation) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: capabilities
  {
    if (msg.capabilities.size() == 0) {
      out << "capabilities: []";
    } else {
      out << "capabilities: [";
      size_t pending_items = msg.capabilities.size();
      for (auto item : msg.capabilities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: workload
  {
    out << "workload: ";
    rosidl_generator_traits::value_to_yaml(msg.workload, out);
    out << ", ";
  }

  // member: failed
  {
    out << "failed: ";
    rosidl_generator_traits::value_to_yaml(msg.failed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.orientation.size() == 0) {
      out << "orientation: []\n";
    } else {
      out << "orientation:\n";
      for (auto item : msg.orientation) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: capabilities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.capabilities.size() == 0) {
      out << "capabilities: []\n";
    } else {
      out << "capabilities:\n";
      for (auto item : msg.capabilities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: workload
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workload: ";
    rosidl_generator_traits::value_to_yaml(msg.workload, out);
    out << "\n";
  }

  // member: failed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failed: ";
    rosidl_generator_traits::value_to_yaml(msg.failed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
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
  const dec_control::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::RobotState & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::RobotState>()
{
  return "dec_control::msg::RobotState";
}

template<>
inline const char * name<dec_control::msg::RobotState>()
{
  return "dec_control/msg/RobotState";
}

template<>
struct has_fixed_size<dec_control::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
