// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__Task __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__Task __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Task_
{
  using Type = Task_<ContainerAllocator>;

  explicit Task_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->execution_time = 0.0f;
      this->requires_collaboration = false;
    }
  }

  explicit Task_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->execution_time = 0.0f;
      this->requires_collaboration = false;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _position_type =
    std::array<float, 3>;
  _position_type position;
  using _capabilities_required_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _capabilities_required_type capabilities_required;
  using _execution_time_type =
    float;
  _execution_time_type execution_time;
  using _prerequisites_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _prerequisites_type prerequisites;
  using _requires_collaboration_type =
    bool;
  _requires_collaboration_type requires_collaboration;

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
  Type & set__capabilities_required(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->capabilities_required = _arg;
    return *this;
  }
  Type & set__execution_time(
    const float & _arg)
  {
    this->execution_time = _arg;
    return *this;
  }
  Type & set__prerequisites(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->prerequisites = _arg;
    return *this;
  }
  Type & set__requires_collaboration(
    const bool & _arg)
  {
    this->requires_collaboration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::Task_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::Task_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::Task_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::Task_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::Task_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::Task_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::Task_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::Task_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::Task_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::Task_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__Task
    std::shared_ptr<dec_control::msg::Task_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__Task
    std::shared_ptr<dec_control::msg::Task_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Task_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->capabilities_required != other.capabilities_required) {
      return false;
    }
    if (this->execution_time != other.execution_time) {
      return false;
    }
    if (this->prerequisites != other.prerequisites) {
      return false;
    }
    if (this->requires_collaboration != other.requires_collaboration) {
      return false;
    }
    return true;
  }
  bool operator!=(const Task_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Task_

// alias to use template instance with default allocator
using Task =
  dec_control::msg::Task_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK__STRUCT_HPP_
