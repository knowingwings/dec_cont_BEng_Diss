// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/TaskList.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_LIST__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__TASK_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tasks'
#include "dec_control/msg/detail/task__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dec_control__msg__TaskList __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__TaskList __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskList_
{
  using Type = TaskList_<ContainerAllocator>;

  explicit TaskList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TaskList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _tasks_type =
    std::vector<dec_control::msg::Task_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dec_control::msg::Task_<ContainerAllocator>>>;
  _tasks_type tasks;

  // setters for named parameter idiom
  Type & set__tasks(
    const std::vector<dec_control::msg::Task_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dec_control::msg::Task_<ContainerAllocator>>> & _arg)
  {
    this->tasks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::msg::TaskList_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::TaskList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::TaskList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::TaskList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::TaskList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::TaskList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::TaskList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::TaskList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::TaskList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::TaskList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__TaskList
    std::shared_ptr<dec_control::msg::TaskList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__TaskList
    std::shared_ptr<dec_control::msg::TaskList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskList_ & other) const
  {
    if (this->tasks != other.tasks) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskList_

// alias to use template instance with default allocator
using TaskList =
  dec_control::msg::TaskList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_LIST__STRUCT_HPP_
