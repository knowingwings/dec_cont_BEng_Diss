// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:srv/GetAvailableTasks.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__TRAITS_HPP_
#define DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/srv/detail/get_available_tasks__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAvailableTasks_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetAvailableTasks_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAvailableTasks_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dec_control

namespace rosidl_generator_traits
{

[[deprecated("use dec_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dec_control::srv::GetAvailableTasks_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::srv::GetAvailableTasks_Request & msg)
{
  return dec_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::srv::GetAvailableTasks_Request>()
{
  return "dec_control::srv::GetAvailableTasks_Request";
}

template<>
inline const char * name<dec_control::srv::GetAvailableTasks_Request>()
{
  return "dec_control/srv/GetAvailableTasks_Request";
}

template<>
struct has_fixed_size<dec_control::srv::GetAvailableTasks_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::srv::GetAvailableTasks_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::srv::GetAvailableTasks_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'available_tasks'
#include "dec_control/msg/detail/task__traits.hpp"

namespace dec_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAvailableTasks_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: available_tasks
  {
    if (msg.available_tasks.size() == 0) {
      out << "available_tasks: []";
    } else {
      out << "available_tasks: [";
      size_t pending_items = msg.available_tasks.size();
      for (auto item : msg.available_tasks) {
        to_flow_style_yaml(item, out);
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
  const GetAvailableTasks_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: available_tasks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.available_tasks.size() == 0) {
      out << "available_tasks: []\n";
    } else {
      out << "available_tasks:\n";
      for (auto item : msg.available_tasks) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAvailableTasks_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dec_control

namespace rosidl_generator_traits
{

[[deprecated("use dec_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dec_control::srv::GetAvailableTasks_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::srv::GetAvailableTasks_Response & msg)
{
  return dec_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::srv::GetAvailableTasks_Response>()
{
  return "dec_control::srv::GetAvailableTasks_Response";
}

template<>
inline const char * name<dec_control::srv::GetAvailableTasks_Response>()
{
  return "dec_control/srv/GetAvailableTasks_Response";
}

template<>
struct has_fixed_size<dec_control::srv::GetAvailableTasks_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::srv::GetAvailableTasks_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::srv::GetAvailableTasks_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dec_control::srv::GetAvailableTasks>()
{
  return "dec_control::srv::GetAvailableTasks";
}

template<>
inline const char * name<dec_control::srv::GetAvailableTasks>()
{
  return "dec_control/srv/GetAvailableTasks";
}

template<>
struct has_fixed_size<dec_control::srv::GetAvailableTasks>
  : std::integral_constant<
    bool,
    has_fixed_size<dec_control::srv::GetAvailableTasks_Request>::value &&
    has_fixed_size<dec_control::srv::GetAvailableTasks_Response>::value
  >
{
};

template<>
struct has_bounded_size<dec_control::srv::GetAvailableTasks>
  : std::integral_constant<
    bool,
    has_bounded_size<dec_control::srv::GetAvailableTasks_Request>::value &&
    has_bounded_size<dec_control::srv::GetAvailableTasks_Response>::value
  >
{
};

template<>
struct is_service<dec_control::srv::GetAvailableTasks>
  : std::true_type
{
};

template<>
struct is_service_request<dec_control::srv::GetAvailableTasks_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dec_control::srv::GetAvailableTasks_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__TRAITS_HPP_
