// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/CapabilityUpdate.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__CapabilityUpdate __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__CapabilityUpdate __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CapabilityUpdate_
{
  using Type = CapabilityUpdate_<ContainerAllocator>;

  explicit CapabilityUpdate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->in_recovery = false;
      this->timestamp = 0ll;
    }
  }

  explicit CapabilityUpdate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->in_recovery = false;
      this->timestamp = 0ll;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _capabilities_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _capabilities_type capabilities;
  using _degradation_mask_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _degradation_mask_type degradation_mask;
  using _in_recovery_type =
    bool;
  _in_recovery_type in_recovery;
  using _timestamp_type =
    int64_t;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__capabilities(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->capabilities = _arg;
    return *this;
  }
  Type & set__degradation_mask(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->degradation_mask = _arg;
    return *this;
  }
  Type & set__in_recovery(
    const bool & _arg)
  {
    this->in_recovery = _arg;
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
    dec_control::msg::CapabilityUpdate_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::CapabilityUpdate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::CapabilityUpdate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::CapabilityUpdate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__CapabilityUpdate
    std::shared_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__CapabilityUpdate
    std::shared_ptr<dec_control::msg::CapabilityUpdate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CapabilityUpdate_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->capabilities != other.capabilities) {
      return false;
    }
    if (this->degradation_mask != other.degradation_mask) {
      return false;
    }
    if (this->in_recovery != other.in_recovery) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const CapabilityUpdate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CapabilityUpdate_

// alias to use template instance with default allocator
using CapabilityUpdate =
  dec_control::msg::CapabilityUpdate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__CAPABILITY_UPDATE__STRUCT_HPP_
