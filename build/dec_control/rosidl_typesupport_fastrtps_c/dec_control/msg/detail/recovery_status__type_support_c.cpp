// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dec_control:msg/RecoveryStatus.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/recovery_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dec_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dec_control/msg/detail/recovery_status__struct.h"
#include "dec_control/msg/detail/recovery_status__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // degraded_capabilities, infeasible_tasks
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // degraded_capabilities, infeasible_tasks
#include "rosidl_runtime_c/string.h"  // recovery_type
#include "rosidl_runtime_c/string_functions.h"  // recovery_type

// forward declare type support functions


using _RecoveryStatus__ros_msg_type = dec_control__msg__RecoveryStatus;

static bool _RecoveryStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RecoveryStatus__ros_msg_type * ros_message = static_cast<const _RecoveryStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    cdr << ros_message->robot_id;
  }

  // Field name: recovery_type
  {
    const rosidl_runtime_c__String * str = &ros_message->recovery_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: degraded_capabilities
  {
    size_t size = ros_message->degraded_capabilities.size;
    auto array_ptr = ros_message->degraded_capabilities.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: infeasible_tasks
  {
    size_t size = ros_message->infeasible_tasks.size;
    auto array_ptr = ros_message->infeasible_tasks.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  return true;
}

static bool _RecoveryStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RecoveryStatus__ros_msg_type * ros_message = static_cast<_RecoveryStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    cdr >> ros_message->robot_id;
  }

  // Field name: recovery_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->recovery_type.data) {
      rosidl_runtime_c__String__init(&ros_message->recovery_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->recovery_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'recovery_type'\n");
      return false;
    }
  }

  // Field name: degraded_capabilities
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->degraded_capabilities.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->degraded_capabilities);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->degraded_capabilities, size)) {
      fprintf(stderr, "failed to create array for field 'degraded_capabilities'");
      return false;
    }
    auto array_ptr = ros_message->degraded_capabilities.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: infeasible_tasks
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->infeasible_tasks.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->infeasible_tasks);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->infeasible_tasks, size)) {
      fprintf(stderr, "failed to create array for field 'infeasible_tasks'");
      return false;
    }
    auto array_ptr = ros_message->infeasible_tasks.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t get_serialized_size_dec_control__msg__RecoveryStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RecoveryStatus__ros_msg_type * ros_message = static_cast<const _RecoveryStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name robot_id
  {
    size_t item_size = sizeof(ros_message->robot_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name recovery_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->recovery_type.size + 1);
  // field.name degraded_capabilities
  {
    size_t array_size = ros_message->degraded_capabilities.size;
    auto array_ptr = ros_message->degraded_capabilities.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name infeasible_tasks
  {
    size_t array_size = ros_message->infeasible_tasks.size;
    auto array_ptr = ros_message->infeasible_tasks.data;
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

static uint32_t _RecoveryStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dec_control__msg__RecoveryStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t max_serialized_size_dec_control__msg__RecoveryStatus(
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

  // member: robot_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: recovery_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: degraded_capabilities
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
  // member: infeasible_tasks
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
    using DataType = dec_control__msg__RecoveryStatus;
    is_plain =
      (
      offsetof(DataType, timestamp) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RecoveryStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dec_control__msg__RecoveryStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RecoveryStatus = {
  "dec_control::msg",
  "RecoveryStatus",
  _RecoveryStatus__cdr_serialize,
  _RecoveryStatus__cdr_deserialize,
  _RecoveryStatus__get_serialized_size,
  _RecoveryStatus__max_serialized_size
};

static rosidl_message_type_support_t _RecoveryStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RecoveryStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, msg, RecoveryStatus)() {
  return &_RecoveryStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
