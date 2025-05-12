// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/task_priority__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskPriority & msg,
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

  // member: priorities
  {
    if (msg.priorities.size() == 0) {
      out << "priorities: []";
    } else {
      out << "priorities: [";
      size_t pending_items = msg.priorities.size();
      for (auto item : msg.priorities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: on_critical_path
  {
    if (msg.on_critical_path.size() == 0) {
      out << "on_critical_path: []";
    } else {
      out << "on_critical_path: [";
      size_t pending_items = msg.on_critical_path.size();
      for (auto item : msg.on_critical_path) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: slack_times
  {
    if (msg.slack_times.size() == 0) {
      out << "slack_times: []";
    } else {
      out << "slack_times: [";
      size_t pending_items = msg.slack_times.size();
      for (auto item : msg.slack_times) {
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
  const TaskPriority & msg,
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

  // member: priorities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.priorities.size() == 0) {
      out << "priorities: []\n";
    } else {
      out << "priorities:\n";
      for (auto item : msg.priorities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: on_critical_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.on_critical_path.size() == 0) {
      out << "on_critical_path: []\n";
    } else {
      out << "on_critical_path:\n";
      for (auto item : msg.on_critical_path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: slack_times
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.slack_times.size() == 0) {
      out << "slack_times: []\n";
    } else {
      out << "slack_times:\n";
      for (auto item : msg.slack_times) {
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

inline std::string to_yaml(const TaskPriority & msg, bool use_flow_style = false)
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
  const dec_control::msg::TaskPriority & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::TaskPriority & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::TaskPriority>()
{
  return "dec_control::msg::TaskPriority";
}

template<>
inline const char * name<dec_control::msg::TaskPriority>()
{
  return "dec_control/msg/TaskPriority";
}

template<>
struct has_fixed_size<dec_control::msg::TaskPriority>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::msg::TaskPriority>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::msg::TaskPriority>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__TRAITS_HPP_
