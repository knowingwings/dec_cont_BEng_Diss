// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__TaskPriority __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__TaskPriority __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskPriority_
{
  using Type = TaskPriority_<ContainerAllocator>;

  explicit TaskPriority_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TaskPriority_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _task_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _task_ids_type task_ids;
  using _priorities_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _priorities_type priorities;
  using _on_critical_path_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _on_critical_path_type on_critical_path;
  using _slack_times_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _slack_times_type slack_times;

  // setters for named parameter idiom
  Type & set__task_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->task_ids = _arg;
    return *this;
  }
  Type & set__priorities(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->priorities = _arg;
    return *this;
  }
  Type & set__on_critical_path(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->on_critical_path = _arg;
    return *this;
  }
  Type & set__slack_times(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->slack_times = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::TaskPriority_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::TaskPriority_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::TaskPriority_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::TaskPriority_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::TaskPriority_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::TaskPriority_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::TaskPriority_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::TaskPriority_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::TaskPriority_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::TaskPriority_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__TaskPriority
    std::shared_ptr<dec_control::msg::TaskPriority_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__TaskPriority
    std::shared_ptr<dec_control::msg::TaskPriority_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskPriority_ & other) const
  {
    if (this->task_ids != other.task_ids) {
      return false;
    }
    if (this->priorities != other.priorities) {
      return false;
    }
    if (this->on_critical_path != other.on_critical_path) {
      return false;
    }
    if (this->slack_times != other.slack_times) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskPriority_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskPriority_

// alias to use template instance with default allocator
using TaskPriority =
  dec_control::msg::TaskPriority_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_PRIORITY__STRUCT_HPP_
