// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/TaskAssignment.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__TaskAssignment __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__TaskAssignment __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskAssignment_
{
  using Type = TaskAssignment_<ContainerAllocator>;

  explicit TaskAssignment_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TaskAssignment_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _task_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _task_ids_type task_ids;
  using _robot_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _robot_ids_type robot_ids;
  using _prices_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _prices_type prices;

  // setters for named parameter idiom
  Type & set__task_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->task_ids = _arg;
    return *this;
  }
  Type & set__robot_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->robot_ids = _arg;
    return *this;
  }
  Type & set__prices(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->prices = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::TaskAssignment_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::TaskAssignment_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::TaskAssignment_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::TaskAssignment_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__TaskAssignment
    std::shared_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__TaskAssignment
    std::shared_ptr<dec_control::msg::TaskAssignment_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskAssignment_ & other) const
  {
    if (this->task_ids != other.task_ids) {
      return false;
    }
    if (this->robot_ids != other.robot_ids) {
      return false;
    }
    if (this->prices != other.prices) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskAssignment_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskAssignment_

// alias to use template instance with default allocator
using TaskAssignment =
  dec_control::msg::TaskAssignment_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__STRUCT_HPP_
