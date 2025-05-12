// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:srv/InitAuction.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__STRUCT_HPP_
#define DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'task'
#include "dec_control/msg/detail/task__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dec_control__srv__InitAuction_Request __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__srv__InitAuction_Request __declspec(deprecated)
#endif

namespace dec_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InitAuction_Request_
{
  using Type = InitAuction_Request_<ContainerAllocator>;

  explicit InitAuction_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task(_init)
  {
    (void)_init;
  }

  explicit InitAuction_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _task_type =
    dec_control::msg::Task_<ContainerAllocator>;
  _task_type task;
  using _capabilities_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _capabilities_type capabilities;

  // setters for named parameter idiom
  Type & set__task(
    const dec_control::msg::Task_<ContainerAllocator> & _arg)
  {
    this->task = _arg;
    return *this;
  }
  Type & set__capabilities(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->capabilities = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::srv::InitAuction_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::srv::InitAuction_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::InitAuction_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::InitAuction_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__srv__InitAuction_Request
    std::shared_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__srv__InitAuction_Request
    std::shared_ptr<dec_control::srv::InitAuction_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InitAuction_Request_ & other) const
  {
    if (this->task != other.task) {
      return false;
    }
    if (this->capabilities != other.capabilities) {
      return false;
    }
    return true;
  }
  bool operator!=(const InitAuction_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InitAuction_Request_

// alias to use template instance with default allocator
using InitAuction_Request =
  dec_control::srv::InitAuction_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dec_control


#ifndef _WIN32
# define DEPRECATED__dec_control__srv__InitAuction_Response __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__srv__InitAuction_Response __declspec(deprecated)
#endif

namespace dec_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InitAuction_Response_
{
  using Type = InitAuction_Response_<ContainerAllocator>;

  explicit InitAuction_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->auction_id = "";
      this->success = false;
    }
  }

  explicit InitAuction_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : auction_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->auction_id = "";
      this->success = false;
    }
  }

  // field types and members
  using _auction_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _auction_id_type auction_id;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__auction_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->auction_id = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::srv::InitAuction_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::srv::InitAuction_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::InitAuction_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::InitAuction_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__srv__InitAuction_Response
    std::shared_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__srv__InitAuction_Response
    std::shared_ptr<dec_control::srv::InitAuction_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InitAuction_Response_ & other) const
  {
    if (this->auction_id != other.auction_id) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const InitAuction_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InitAuction_Response_

// alias to use template instance with default allocator
using InitAuction_Response =
  dec_control::srv::InitAuction_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dec_control

namespace dec_control
{

namespace srv
{

struct InitAuction
{
  using Request = dec_control::srv::InitAuction_Request;
  using Response = dec_control::srv::InitAuction_Response;
};

}  // namespace srv

}  // namespace dec_control

#endif  // DEC_CONTROL__SRV__DETAIL__INIT_AUCTION__STRUCT_HPP_
