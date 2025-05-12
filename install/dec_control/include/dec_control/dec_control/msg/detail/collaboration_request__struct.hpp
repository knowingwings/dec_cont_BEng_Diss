// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dec_control:msg/CollaborationRequest.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__STRUCT_HPP_
#define DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dec_control__msg__CollaborationRequest __attribute__((deprecated))
#else
# define DEPRECATED__dec_control__msg__CollaborationRequest __declspec(deprecated)
#endif

namespace dec_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CollaborationRequest_
{
  using Type = CollaborationRequest_<ContainerAllocator>;

  explicit CollaborationRequest_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = 0l;
      this->robot_id = 0l;
      this->leadership_score = 0.0;
      this->timestamp = 0ll;
    }
  }

  explicit CollaborationRequest_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = 0l;
      this->robot_id = 0l;
      this->leadership_score = 0.0;
      this->timestamp = 0ll;
    }
  }

  // field types and members
  using _task_id_type =
    int32_t;
  _task_id_type task_id;
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _leadership_score_type =
    double;
  _leadership_score_type leadership_score;
  using _timestamp_type =
    int64_t;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__task_id(
    const int32_t & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__leadership_score(
    const double & _arg)
  {
    this->leadership_score = _arg;
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
    dec_control::msg::CollaborationRequest_<ContainerAllocator> *;
  using ConstRawPtr =
    const dec_control::msg::CollaborationRequest_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::CollaborationRequest_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dec_control::msg::CollaborationRequest_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dec_control__msg__CollaborationRequest
    std::shared_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dec_control__msg__CollaborationRequest
    std::shared_ptr<dec_control::msg::CollaborationRequest_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CollaborationRequest_ & other) const
  {
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->leadership_score != other.leadership_score) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const CollaborationRequest_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CollaborationRequest_

// alias to use template instance with default allocator
using CollaborationRequest =
  dec_control::msg::CollaborationRequest_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dec_control

#endif  // DEC_CONTROL__MSG__DETAIL__COLLABORATION_REQUEST__STRUCT_HPP_
