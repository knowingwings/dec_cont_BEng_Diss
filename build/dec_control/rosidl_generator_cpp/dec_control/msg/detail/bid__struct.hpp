// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/Bid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__BID__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__BID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__Bid __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__Bid __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bid_
{
  using Type = Bid_<ContainerAllocator>;

  explicit Bid_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->task_id = 0l;
      this->bid_value = 0.0;
      this->utility = 0.0;
    }
  }

  explicit Bid_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->task_id = 0l;
      this->bid_value = 0.0;
      this->utility = 0.0;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _task_id_type =
    int32_t;
  _task_id_type task_id;
  using _bid_value_type =
    double;
  _bid_value_type bid_value;
  using _utility_type =
    double;
  _utility_type utility;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__task_id(
    const int32_t & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__bid_value(
    const double & _arg)
  {
    this->bid_value = _arg;
    return *this;
  }
  Type & set__utility(
    const double & _arg)
  {
    this->utility = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::Bid_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::Bid_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::Bid_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::Bid_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::Bid_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::Bid_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::Bid_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::Bid_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::Bid_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::Bid_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__Bid
    std::shared_ptr<dec_control::msg::Bid_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__Bid
    std::shared_ptr<dec_control::msg::Bid_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bid_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->bid_value != other.bid_value) {
      return false;
    }
    if (this->utility != other.utility) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bid_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bid_

// alias to use template instance with default allocator
using Bid =
  dec_control::msg::Bid_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__BID__STRUCT_HPP_
