// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dec_control:msg/NetworkTopology.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/network_topology__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dec_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dec_control/msg/detail/network_topology__struct.h"
#include "dec_control/msg/detail/network_topology__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // adjacency_matrix, link_latency, link_quality, link_reliability
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // adjacency_matrix, link_latency, link_quality, link_reliability

// forward declare type support functions


using _NetworkTopology__ros_msg_type = dec_control__msg__NetworkTopology;

static bool _NetworkTopology__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _NetworkTopology__ros_msg_type * ros_message = static_cast<const _NetworkTopology__ros_msg_type *>(untyped_ros_message);
  // Field name: num_robots
  {
    cdr << ros_message->num_robots;
  }

  // Field name: adjacency_matrix
  {
    size_t size = ros_message->adjacency_matrix.size;
    auto array_ptr = ros_message->adjacency_matrix.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: link_quality
  {
    size_t size = ros_message->link_quality.size;
    auto array_ptr = ros_message->link_quality.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: link_latency
  {
    size_t size = ros_message->link_latency.size;
    auto array_ptr = ros_message->link_latency.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: link_reliability
  {
    size_t size = ros_message->link_reliability.size;
    auto array_ptr = ros_message->link_reliability.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  return true;
}

static bool _NetworkTopology__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _NetworkTopology__ros_msg_type * ros_message = static_cast<_NetworkTopology__ros_msg_type *>(untyped_ros_message);
  // Field name: num_robots
  {
    cdr >> ros_message->num_robots;
  }

  // Field name: adjacency_matrix
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->adjacency_matrix.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->adjacency_matrix);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->adjacency_matrix, size)) {
      fprintf(stderr, "failed to create array for field 'adjacency_matrix'");
      return false;
    }
    auto array_ptr = ros_message->adjacency_matrix.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: link_quality
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->link_quality.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->link_quality);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->link_quality, size)) {
      fprintf(stderr, "failed to create array for field 'link_quality'");
      return false;
    }
    auto array_ptr = ros_message->link_quality.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: link_latency
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->link_latency.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->link_latency);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->link_latency, size)) {
      fprintf(stderr, "failed to create array for field 'link_latency'");
      return false;
    }
    auto array_ptr = ros_message->link_latency.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: link_reliability
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->link_reliability.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->link_reliability);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->link_reliability, size)) {
      fprintf(stderr, "failed to create array for field 'link_reliability'");
      return false;
    }
    auto array_ptr = ros_message->link_reliability.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t get_serialized_size_dec_control__msg__NetworkTopology(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _NetworkTopology__ros_msg_type * ros_message = static_cast<const _NetworkTopology__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name num_robots
  {
    size_t item_size = sizeof(ros_message->num_robots);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name adjacency_matrix
  {
    size_t array_size = ros_message->adjacency_matrix.size;
    auto array_ptr = ros_message->adjacency_matrix.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name link_quality
  {
    size_t array_size = ros_message->link_quality.size;
    auto array_ptr = ros_message->link_quality.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name link_latency
  {
    size_t array_size = ros_message->link_latency.size;
    auto array_ptr = ros_message->link_latency.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name link_reliability
  {
    size_t array_size = ros_message->link_reliability.size;
    auto array_ptr = ros_message->link_reliability.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _NetworkTopology__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dec_control__msg__NetworkTopology(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t max_serialized_size_dec_control__msg__NetworkTopology(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: num_robots
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: adjacency_matrix
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: link_quality
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: link_latency
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: link_reliability
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: timestamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dec_control__msg__NetworkTopology;
    is_plain =
      (
      offsetof(DataType, timestamp) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _NetworkTopology__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dec_control__msg__NetworkTopology(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_NetworkTopology = {
  "dec_control::msg",
  "NetworkTopology",
  _NetworkTopology__cdr_serialize,
  _NetworkTopology__cdr_deserialize,
  _NetworkTopology__get_serialized_size,
  _NetworkTopology__max_serialized_size
};

static rosidl_message_type_support_t _NetworkTopology__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_NetworkTopology,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, msg, NetworkTopology)() {
  return &_NetworkTopology__type_support;
}

#if defined(__cplusplus)
}
#endif
