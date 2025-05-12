// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__BUILDER_HPP_
#define DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dec_control/msg/detail/network_topology__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dec_control
{

namespace msg
{

namespace builder
{

class Init_NetworkTopology_timestamp
{
public:
  explicit Init_NetworkTopology_timestamp(::dec_control::msg::NetworkTopology & msg)
  : msg_(msg)
  {}
  ::dec_control::msg::NetworkTopology timestamp(::dec_control::msg::NetworkTopology::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dec_control::msg::NetworkTopology msg_;
};

class Init_NetworkTopology_link_reliability
{
public:
  explicit Init_NetworkTopology_link_reliability(::dec_control::msg::NetworkTopology & msg)
  : msg_(msg)
  {}
  Init_NetworkTopology_timestamp link_reliability(::dec_control::msg::NetworkTopology::_link_reliability_type arg)
  {
    msg_.link_reliability = std::move(arg);
    return Init_NetworkTopology_timestamp(msg_);
  }

private:
  ::dec_control::msg::NetworkTopology msg_;
};

class Init_NetworkTopology_link_latency
{
public:
  explicit Init_NetworkTopology_link_latency(::dec_control::msg::NetworkTopology & msg)
  : msg_(msg)
  {}
  Init_NetworkTopology_link_reliability link_latency(::dec_control::msg::NetworkTopology::_link_latency_type arg)
  {
    msg_.link_latency = std::move(arg);
    return Init_NetworkTopology_link_reliability(msg_);
  }

private:
  ::dec_control::msg::NetworkTopology msg_;
};

class Init_NetworkTopology_link_quality
{
public:
  explicit Init_NetworkTopology_link_quality(::dec_control::msg::NetworkTopology & msg)
  : msg_(msg)
  {}
  Init_NetworkTopology_link_latency link_quality(::dec_control::msg::NetworkTopology::_link_quality_type arg)
  {
    msg_.link_quality = std::move(arg);
    return Init_NetworkTopology_link_latency(msg_);
  }

private:
  ::dec_control::msg::NetworkTopology msg_;
};

class Init_NetworkTopology_adjacency_matrix
{
public:
  explicit Init_NetworkTopology_adjacency_matrix(::dec_control::msg::NetworkTopology & msg)
  : msg_(msg)
  {}
  Init_NetworkTopology_link_quality adjacency_matrix(::dec_control::msg::NetworkTopology::_adjacency_matrix_type arg)
  {
    msg_.adjacency_matrix = std::move(arg);
    return Init_NetworkTopology_link_quality(msg_);
  }

private:
  ::dec_control::msg::NetworkTopology msg_;
};

class Init_NetworkTopology_num_robots
{
public:
  Init_NetworkTopology_num_robots()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NetworkTopology_adjacency_matrix num_robots(::dec_control::msg::NetworkTopology::_num_robots_type arg)
  {
    msg_.num_robots = std::move(arg);
    return Init_NetworkTopology_adjacency_matrix(msg_);
  }

private:
  ::dec_control::msg::NetworkTopology msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dec_control::msg::NetworkTopology>()
{
  return dec_control::msg::builder::Init_NetworkTopology_num_robots();
}

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__BUILDER_HPP_
