// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:srv/SubmitBid.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__STRUCT_HPP_
#define DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__srv__SubmitBid_Request __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__srv__SubmitBid_Request __declspec(deprecated)
#endif

namespace dec_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SubmitBid_Request_
{
  using Type = SubmitBid_Request_<ContainerAllocator>;

  explicit SubmitBid_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->auction_id = "";
      this->robot_id = "";
      this->bid_value = 0.0f;
    }
  }

  explicit SubmitBid_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : auction_id(_alloc),
    robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->auction_id = "";
      this->robot_id = "";
      this->bid_value = 0.0f;
    }
  }

  // field types and members
  using _auction_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _auction_id_type auction_id;
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _bid_value_type =
    float;
  _bid_value_type bid_value;
  using _resource_availability_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _resource_availability_type resource_availability;

  // setters for named parameter idiom
  Type & set__auction_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->auction_id = _arg;
    return *this;
  }
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__bid_value(
    const float & _arg)
  {
    this->bid_value = _arg;
    return *this;
  }
  Type & set__resource_availability(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->resource_availability = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::srv::SubmitBid_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::srv::SubmitBid_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::SubmitBid_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::SubmitBid_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__srv__SubmitBid_Request
    std::shared_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__srv__SubmitBid_Request
    std::shared_ptr<dec_control::srv::SubmitBid_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SubmitBid_Request_ & other) const
  {
    if (this->auction_id != other.auction_id) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->bid_value != other.bid_value) {
      return false;
    }
    if (this->resource_availability != other.resource_availability) {
      return false;
    }
    return true;
  }
  bool operator!=(const SubmitBid_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SubmitBid_Request_

// alias to use template instance with default allocator
using SubmitBid_Request =
  dec_control::srv::SubmitBid_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dec_control


#ifndef _WIN32
# define DEPRECATED__dec_control__srv__SubmitBid_Response __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__srv__SubmitBid_Response __declspec(deprecated)
#endif

namespace dec_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SubmitBid_Response_
{
  using Type = SubmitBid_Response_<ContainerAllocator>;

  explicit SubmitBid_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit SubmitBid_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dec_control::srv::SubmitBid_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::srv::SubmitBid_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::SubmitBid_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::srv::SubmitBid_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__srv__SubmitBid_Response
    std::shared_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__srv__SubmitBid_Response
    std::shared_ptr<dec_control::srv::SubmitBid_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SubmitBid_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const SubmitBid_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SubmitBid_Response_

// alias to use template instance with default allocator
using SubmitBid_Response =
  dec_control::srv::SubmitBid_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dec_control

namespace dec_control
{

namespace srv
{

struct SubmitBid
{
  using Request = dec_control::srv::SubmitBid_Request;
  using Response = dec_control::srv::SubmitBid_Response;
};

}  // namespace srv

}  // namespace dec_control

#endif  // DEC_CONTROL__SRV__DETAIL__SUBMIT_BID__STRUCT_HPP_
