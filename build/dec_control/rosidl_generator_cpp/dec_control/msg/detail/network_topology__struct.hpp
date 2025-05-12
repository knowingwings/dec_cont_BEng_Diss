// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__NetworkTopology __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__NetworkTopology __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NetworkTopology_
{
  using Type = NetworkTopology_<ContainerAllocator>;

  explicit NetworkTopology_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_robots = 0l;
      this->timestamp = 0ll;
    }
  }

  explicit NetworkTopology_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_robots = 0l;
      this->timestamp = 0ll;
    }
  }

  // field types and members
  using _num_robots_type =
    int32_t;
  _num_robots_type num_robots;
  using _adjacency_matrix_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _adjacency_matrix_type adjacency_matrix;
  using _link_quality_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _link_quality_type link_quality;
  using _link_latency_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _link_latency_type link_latency;
  using _link_reliability_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _link_reliability_type link_reliability;
  using _timestamp_type =
    int64_t;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__num_robots(
    const int32_t & _arg)
  {
    this->num_robots = _arg;
    return *this;
  }
  Type & set__adjacency_matrix(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->adjacency_matrix = _arg;
    return *this;
  }
  Type & set__link_quality(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->link_quality = _arg;
    return *this;
  }
  Type & set__link_latency(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->link_latency = _arg;
    return *this;
  }
  Type & set__link_reliability(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->link_reliability = _arg;
    return *this;
  }
  Type & set__timestamp(
    const int64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::NetworkTopology_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::NetworkTopology_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::NetworkTopology_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::NetworkTopology_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__NetworkTopology
    std::shared_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__NetworkTopology
    std::shared_ptr<dec_control::msg::NetworkTopology_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NetworkTopology_ & other) const
  {
    if (this->num_robots != other.num_robots) {
      return false;
    }
    if (this->adjacency_matrix != other.adjacency_matrix) {
      return false;
    }
    if (this->link_quality != other.link_quality) {
      return false;
    }
    if (this->link_latency != other.link_latency) {
      return false;
    }
    if (this->link_reliability != other.link_reliability) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const NetworkTopology_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NetworkTopology_

// alias to use template instance with default allocator
using NetworkTopology =
  dec_control::msg::NetworkTopology_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__NETWORK_TOPOLOGY__STRUCT_HPP_
