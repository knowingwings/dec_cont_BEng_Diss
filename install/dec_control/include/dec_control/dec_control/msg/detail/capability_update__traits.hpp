// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/CapabilityUpdate.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/capability_update__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const CapabilityUpdate & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
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

  // member: degradation_mask
  {
    if (msg.degradation_mask.size() == 0) {
      out << "degradation_mask: []";
    } else {
      out << "degradation_mask: [";
      size_t pending_items = msg.degradation_mask.size();
      for (auto item : msg.degradation_mask) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: in_recovery
  {
    out << "in_recovery: ";
    rosidl_generator_traits::value_to_yaml(msg.in_recovery, out);
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
  const CapabilityUpdate & msg,
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

  // member: degradation_mask
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.degradation_mask.size() == 0) {
      out << "degradation_mask: []\n";
    } else {
      out << "degradation_mask:\n";
      for (auto item : msg.degradation_mask) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: in_recovery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_recovery: ";
    rosidl_generator_traits::value_to_yaml(msg.in_recovery, out);
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

inline std::string to_yaml(const CapabilityUpdate & msg, bool use_flow_style = false)
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
  const dec_control::msg::CapabilityUpdate & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::CapabilityUpdate & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::CapabilityUpdate>()
{
  return "dec_control::msg::CapabilityUpdate";
}

template<>
inline const char * name<dec_control::msg::CapabilityUpdate>()
{
  return "dec_control/msg/CapabilityUpdate";
}

template<>
struct has_fixed_size<dec_control::msg::CapabilityUpdate>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::msg::CapabilityUpdate>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::msg::CapabilityUpdate>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__TRAITS_HPP_
