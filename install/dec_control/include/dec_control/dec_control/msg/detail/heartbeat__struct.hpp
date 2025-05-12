// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__HEARTBEAT__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__HEARTBEAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__Heartbeat __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__Heartbeat __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Heartbeat_
{
  using Type = Heartbeat_<ContainerAllocator>;

  explicit Heartbeat_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->timestamp = 0ll;
      this->status = 0;
    }
  }

  explicit Heartbeat_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->timestamp = 0ll;
      this->status = 0;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _timestamp_type =
    int64_t;
  _timestamp_type timestamp;
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__timestamp(
    const int64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::Heartbeat_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::Heartbeat_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::Heartbeat_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::Heartbeat_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::Heartbeat_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::Heartbeat_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::Heartbeat_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::Heartbeat_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::Heartbeat_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::Heartbeat_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__Heartbeat
    std::shared_ptr<dec_control::msg::Heartbeat_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__Heartbeat
    std::shared_ptr<dec_control::msg::Heartbeat_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Heartbeat_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Heartbeat_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Heartbeat_

// alias to use template instance with default allocator
using Heartbeat =
  dec_control::msg::Heartbeat_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__HEARTBEAT__STRUCT_HPP_
