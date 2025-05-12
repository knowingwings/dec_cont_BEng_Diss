// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Task & msg,
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

  // member: capabilities_required
  {
    if (msg.capabilities_required.size() == 0) {
      out << "capabilities_required: []";
    } else {
      out << "capabilities_required: [";
      size_t pending_items = msg.capabilities_required.size();
      for (auto item : msg.capabilities_required) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: execution_time
  {
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
    out << ", ";
  }

  // member: prerequisites
  {
    if (msg.prerequisites.size() == 0) {
      out << "prerequisites: []";
    } else {
      out << "prerequisites: [";
      size_t pending_items = msg.prerequisites.size();
      for (auto item : msg.prerequisites) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: requires_collaboration
  {
    out << "requires_collaboration: ";
    rosidl_generator_traits::value_to_yaml(msg.requires_collaboration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Task & msg,
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

  // member: capabilities_required
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.capabilities_required.size() == 0) {
      out << "capabilities_required: []\n";
    } else {
      out << "capabilities_required:\n";
      for (auto item : msg.capabilities_required) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: execution_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
    out << "\n";
  }

  // member: prerequisites
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.prerequisites.size() == 0) {
      out << "prerequisites: []\n";
    } else {
      out << "prerequisites:\n";
      for (auto item : msg.prerequisites) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: requires_collaboration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requires_collaboration: ";
    rosidl_generator_traits::value_to_yaml(msg.requires_collaboration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Task & msg, bool use_flow_style = false)
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
  const dec_control::msg::Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::Task & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::Task>()
{
  return "dec_control::msg::Task";
}

template<>
inline const char * name<dec_control::msg::Task>()
{
  return "dec_control/msg/Task";
}

template<>
struct has_fixed_size<dec_control::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::msg::Task>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__TASK__TRAITS_HPP_
