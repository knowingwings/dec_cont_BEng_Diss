// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__RecoveryStatus __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__RecoveryStatus __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RecoveryStatus_
{
  using Type = RecoveryStatus_<ContainerAllocator>;

  explicit RecoveryStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->recovery_type = "";
      this->timestamp = 0ll;
    }
  }

  explicit RecoveryStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : recovery_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->recovery_type = "";
      this->timestamp = 0ll;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _recovery_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _recovery_type_type recovery_type;
  using _degraded_capabilities_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _degraded_capabilities_type degraded_capabilities;
  using _infeasible_tasks_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _infeasible_tasks_type infeasible_tasks;
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
  Type & set__recovery_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->recovery_type = _arg;
    return *this;
  }
  Type & set__degraded_capabilities(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->degraded_capabilities = _arg;
    return *this;
  }
  Type & set__infeasible_tasks(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->infeasible_tasks = _arg;
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
    dec_control::msg::RecoveryStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::RecoveryStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::RecoveryStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::RecoveryStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__RecoveryStatus
    std::shared_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__RecoveryStatus
    std::shared_ptr<dec_control::msg::RecoveryStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RecoveryStatus_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->recovery_type != other.recovery_type) {
      return false;
    }
    if (this->degraded_capabilities != other.degraded_capabilities) {
      return false;
    }
    if (this->infeasible_tasks != other.infeasible_tasks) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const RecoveryStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RecoveryStatus_

// alias to use template instance with default allocator
using RecoveryStatus =
  dec_control::msg::RecoveryStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__RECOVERY_STATUS__STRUCT_HPP_
