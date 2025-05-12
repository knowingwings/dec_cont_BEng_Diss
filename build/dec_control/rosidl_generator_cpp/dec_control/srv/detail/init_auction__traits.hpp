// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:srv/InitAuction.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__TRAITS_HPP_
#define DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/srv/detail/init_auction__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'task'
#include "dec_control/msg/detail/task__traits.hpp"

namespace dec_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const InitAuction_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: task
  {
    out << "task: ";
    to_flow_style_yaml(msg.task, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InitAuction_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task:\n";
    to_block_style_yaml(msg.task, out, indentation + 2);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InitAuction_Request & msg, bool use_flow_style = false)
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
  const dec_control::srv::InitAuction_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::srv::InitAuction_Request & msg)
{
  return dec_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::srv::InitAuction_Request>()
{
  return "dec_control::srv::InitAuction_Request";
}

template<>
inline const char * name<dec_control::srv::InitAuction_Request>()
{
  return "dec_control/srv/InitAuction_Request";
}

template<>
struct has_fixed_size<dec_control::srv::InitAuction_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::srv::InitAuction_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::srv::InitAuction_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dec_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const InitAuction_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: auction_id
  {
    out << "auction_id: ";
    rosidl_generator_traits::value_to_yaml(msg.auction_id, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InitAuction_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: auction_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "auction_id: ";
    rosidl_generator_traits::value_to_yaml(msg.auction_id, out);
    out << "\n";
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InitAuction_Response & msg, bool use_flow_style = false)
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
  const dec_control::srv::InitAuction_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::srv::InitAuction_Response & msg)
{
  return dec_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::srv::InitAuction_Response>()
{
  return "dec_control::srv::InitAuction_Response";
}

template<>
inline const char * name<dec_control::srv::InitAuction_Response>()
{
  return "dec_control/srv/InitAuction_Response";
}

template<>
struct has_fixed_size<dec_control::srv::InitAuction_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::srv::InitAuction_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::srv::InitAuction_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dec_control::srv::InitAuction>()
{
  return "dec_control::srv::InitAuction";
}

template<>
inline const char * name<dec_control::srv::InitAuction>()
{
  return "dec_control/srv/InitAuction";
}

template<>
struct has_fixed_size<dec_control::srv::InitAuction>
  : std::integral_constant<
    bool,
    has_fixed_size<dec_control::srv::InitAuction_Request>::value &&
    has_fixed_size<dec_control::srv::InitAuction_Response>::value
  >
{
};

template<>
struct has_bounded_size<dec_control::srv::InitAuction>
  : std::integral_constant<
    bool,
    has_bounded_size<dec_control::srv::InitAuction_Request>::value &&
    has_bounded_size<dec_control::srv::InitAuction_Response>::value
  >
{
};

template<>
struct is_service<dec_control::srv::InitAuction>
  : std::true_type
{
};

template<>
struct is_service_request<dec_control::srv::InitAuction_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dec_control::srv::InitAuction_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__TRAITS_HPP_
