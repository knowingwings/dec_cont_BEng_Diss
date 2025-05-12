// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__TRAITS_HPP_
#define DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dec_control/msg/detail/network_topology__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dec_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const NetworkTopology & msg,
  std::ostream & out)
{
  out << "{";
  // member: num_robots
  {
    out << "num_robots: ";
    rosidl_generator_traits::value_to_yaml(msg.num_robots, out);
    out << ", ";
  }

  // member: adjacency_matrix
  {
    if (msg.adjacency_matrix.size() == 0) {
      out << "adjacency_matrix: []";
    } else {
      out << "adjacency_matrix: [";
      size_t pending_items = msg.adjacency_matrix.size();
      for (auto item : msg.adjacency_matrix) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: link_quality
  {
    if (msg.link_quality.size() == 0) {
      out << "link_quality: []";
    } else {
      out << "link_quality: [";
      size_t pending_items = msg.link_quality.size();
      for (auto item : msg.link_quality) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: link_latency
  {
    if (msg.link_latency.size() == 0) {
      out << "link_latency: []";
    } else {
      out << "link_latency: [";
      size_t pending_items = msg.link_latency.size();
      for (auto item : msg.link_latency) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: link_reliability
  {
    if (msg.link_reliability.size() == 0) {
      out << "link_reliability: []";
    } else {
      out << "link_reliability: [";
      size_t pending_items = msg.link_reliability.size();
      for (auto item : msg.link_reliability) {
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
  const NetworkTopology & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num_robots
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_robots: ";
    rosidl_generator_traits::value_to_yaml(msg.num_robots, out);
    out << "\n";
  }

  // member: adjacency_matrix
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.adjacency_matrix.size() == 0) {
      out << "adjacency_matrix: []\n";
    } else {
      out << "adjacency_matrix:\n";
      for (auto item : msg.adjacency_matrix) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: link_quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.link_quality.size() == 0) {
      out << "link_quality: []\n";
    } else {
      out << "link_quality:\n";
      for (auto item : msg.link_quality) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: link_latency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.link_latency.size() == 0) {
      out << "link_latency: []\n";
    } else {
      out << "link_latency:\n";
      for (auto item : msg.link_latency) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: link_reliability
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.link_reliability.size() == 0) {
      out << "link_reliability: []\n";
    } else {
      out << "link_reliability:\n";
      for (auto item : msg.link_reliability) {
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

inline std::string to_yaml(const NetworkTopology & msg, bool use_flow_style = false)
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
  const dec_control::msg::NetworkTopology & msg,
  std::ostream & out, size_t indentation = 0)
{
  dec_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dec_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const dec_control::msg::NetworkTopology & msg)
{
  return dec_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dec_control::msg::NetworkTopology>()
{
  return "dec_control::msg::NetworkTopology";
}

template<>
inline const char * name<dec_control::msg::NetworkTopology>()
{
  return "dec_control/msg/NetworkTopology";
}

template<>
struct has_fixed_size<dec_control::msg::NetworkTopology>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dec_control::msg::NetworkTopology>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dec_control::msg::NetworkTopology>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__TRAITS_HPP_
