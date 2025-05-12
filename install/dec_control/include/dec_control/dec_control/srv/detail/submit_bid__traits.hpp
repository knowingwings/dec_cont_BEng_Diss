// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:srv/SubmitBid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__TRAITS_HPP_
#define DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/srv/detail/submit_bid__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const SubmitBid_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: auction_id
  {
    out << "auction_id: ";
    rosidl_generator_traits::value_to_yaml(msg.auction_id, out);
    out << ", ";
  }

  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: bid_value
  {
    out << "bid_value: ";
    rosidl_generator_traits::value_to_yaml(msg.bid_value, out);
    out << ", ";
  }

  // member: resource_availability
  {
    if (msg.resource_availability.size() == 0) {
      out << "resource_availability: []";
    } else {
      out << "resource_availability: [";
      size_t pending_items = msg.resource_availability.size();
      for (auto item : msg.resource_availability) {
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
  const SubmitBid_Request & msg,
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

  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
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

  // member: resource_availability
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.resource_availability.size() == 0) {
      out << "resource_availability: []\n";
    } else {
      out << "resource_availability:\n";
      for (auto item : msg.resource_availability) {
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

inline std::string to_yaml(const SubmitBid_Request & msg, bool use_flow_style = false)
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
  const dec_control::srv::SubmitBid_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::srv::SubmitBid_Request & msg)
{
  return dec_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::srv::SubmitBid_Request>()
{
  return "dec_control::srv::SubmitBid_Request";
}

template<>
inline const char * name<dec_control::srv::SubmitBid_Request>()
{
  return "dec_control/srv/SubmitBid_Request";
}

template<>
struct has_fixed_size<dec_control::srv::SubmitBid_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::srv::SubmitBid_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::srv::SubmitBid_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dec_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const SubmitBid_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SubmitBid_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SubmitBid_Response & msg, bool use_flow_style = false)
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
  const dec_control::srv::SubmitBid_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::srv::SubmitBid_Response & msg)
{
  return dec_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::srv::SubmitBid_Response>()
{
  return "dec_control::srv::SubmitBid_Response";
}

template<>
inline const char * name<dec_control::srv::SubmitBid_Response>()
{
  return "dec_control/srv/SubmitBid_Response";
}

template<>
struct has_fixed_size<dec_control::srv::SubmitBid_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dec_control::srv::SubmitBid_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dec_control::srv::SubmitBid_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dec_control::srv::SubmitBid>()
{
  return "dec_control::srv::SubmitBid";
}

template<>
inline const char * name<dec_control::srv::SubmitBid>()
{
  return "dec_control/srv/SubmitBid";
}

template<>
struct has_fixed_size<dec_control::srv::SubmitBid>
  : std::integral_constant<
    bool,
    has_fixed_size<dec_control::srv::SubmitBid_Request>::value &&
    has_fixed_size<dec_control::srv::SubmitBid_Response>::value
  >
{
};

template<>
struct has_bounded_size<dec_control::srv::SubmitBid>
  : std::integral_constant<
    bool,
    has_bounded_size<dec_control::srv::SubmitBid_Request>::value &&
    has_bounded_size<dec_control::srv::SubmitBid_Response>::value
  >
{
};

template<>
struct is_service<dec_control::srv::SubmitBid>
  : std::true_type
{
};

template<>
struct is_service_request<dec_control::srv::SubmitBid_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dec_control::srv::SubmitBid_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__TRAITS_HPP_
