// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__RobotState __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->orientation.begin(), this->orientation.end(), 0.0f);
      this->workload = 0.0;
      this->failed = false;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc),
    orientation(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->orientation.begin(), this->orientation.end(), 0.0f);
      this->workload = 0.0;
      this->failed = false;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _position_type =
    std::array<float, 3>;
  _position_type position;
  using _orientation_type =
    std::array<float, 3>;
  _orientation_type orientation;
  using _capabilities_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _capabilities_type capabilities;
  using _workload_type =
    double;
  _workload_type workload;
  using _failed_type =
    bool;
  _failed_type failed;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<float, 3> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__orientation(
    const std::array<float, 3> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__capabilities(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->capabilities = _arg;
    return *this;
  }
  Type & set__workload(
    const double & _arg)
  {
    this->workload = _arg;
    return *this;
  }
  Type & set__failed(
    const bool & _arg)
  {
    this->failed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__RobotState
    std::shared_ptr<dec_control::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__RobotState
    std::shared_ptr<dec_control::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->capabilities != other.capabilities) {
      return false;
    }
    if (this->workload != other.workload) {
      return false;
    }
    if (this->failed != other.failed) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  dec_control::msg::RobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
