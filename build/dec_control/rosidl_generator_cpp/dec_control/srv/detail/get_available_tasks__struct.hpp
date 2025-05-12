// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:srv/GetAvailableTasks.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__STRUCT_HPP_
#define DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__srv__GetAvailableTasks_Request __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__srv__GetAvailableTasks_Request __declspec(deprecated)
#endif

namespace dec_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAvailableTasks_Request_
{
  using Type = GetAvailableTasks_Request_<ContainerAllocator>;

  explicit GetAvailableTasks_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  explicit GetAvailableTasks_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  // field types and members
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;

  // setters for named parameter idiom
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__srv__GetAvailableTasks_Request
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__srv__GetAvailableTasks_Request
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAvailableTasks_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAvailableTasks_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAvailableTasks_Request_

// alias to use template instance with default allocator
using GetAvailableTasks_Request =
  dec_control::srv::GetAvailableTasks_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dec_control


// Include directives for member types
// Member 'available_tasks'
#include "dec_control/msg/detail/task__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dec_control__srv__GetAvailableTasks_Response __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__srv__GetAvailableTasks_Response __declspec(deprecated)
#endif

namespace dec_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAvailableTasks_Response_
{
  using Type = GetAvailableTasks_Response_<ContainerAllocator>;

  explicit GetAvailableTasks_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetAvailableTasks_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _available_tasks_type =
    std::vector<dec_control::msg::Task_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dec_control::msg::Task_<ContainerAllocator>>>;
  _available_tasks_type available_tasks;

  // setters for named parameter idiom
  Type & set__available_tasks(
    const std::vector<dec_control::msg::Task_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dec_control::msg::Task_<ContainerAllocator>>> & _arg)
  {
    this->available_tasks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__srv__GetAvailableTasks_Response
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__srv__GetAvailableTasks_Response
    std::shared_ptr<dec_control::srv::GetAvailableTasks_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAvailableTasks_Response_ & other) const
  {
    if (this->available_tasks != other.available_tasks) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAvailableTasks_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAvailableTasks_Response_

// alias to use template instance with default allocator
using GetAvailableTasks_Response =
  dec_control::srv::GetAvailableTasks_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dec_control

namespace dec_control
{

namespace srv
{

struct GetAvailableTasks
{
  using Request = dec_control::srv::GetAvailableTasks_Request;
  using Response = dec_control::srv::GetAvailableTasks_Response;
};

}  // namespace srv

}  // namespace dec_control

#endif  // DEC_CONTROL__SRV__DETAIL__GET_AVAILABLE_TASKS__STRUCT_HPP_
