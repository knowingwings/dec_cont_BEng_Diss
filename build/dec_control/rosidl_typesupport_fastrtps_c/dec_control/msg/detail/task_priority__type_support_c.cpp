// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dec_control:msg/TaskPriority.idl
// generated code does not contain a copyright notice
#include "dec_control/msg/detail/task_priority__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dec_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dec_control/msg/detail/task_priority__struct.h"
#include "dec_control/msg/detail/task_priority__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // on_critical_path, priorities, slack_times, task_ids
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // on_critical_path, priorities, slack_times, task_ids

// forward declare type support functions


using _TaskPriority__ros_msg_type = dec_control__msg__TaskPriority;

static bool _TaskPriority__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TaskPriority__ros_msg_type * ros_message = static_cast<const _TaskPriority__ros_msg_type *>(untyped_ros_message);
  // Field name: task_ids
  {
    size_t size = ros_message->task_ids.size;
    auto array_ptr = ros_message->task_ids.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: priorities
  {
    size_t size = ros_message->priorities.size;
    auto array_ptr = ros_message->priorities.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: on_critical_path
  {
    size_t size = ros_message->on_critical_path.size;
    auto array_ptr = ros_message->on_critical_path.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: slack_times
  {
    size_t size = ros_message->slack_times.size;
    auto array_ptr = ros_message->slack_times.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _TaskPriority__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TaskPriority__ros_msg_type * ros_message = static_cast<_TaskPriority__ros_msg_type *>(untyped_ros_message);
  // Field name: task_ids
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->task_ids.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->task_ids);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->task_ids, size)) {
      fprintf(stderr, "failed to create array for field 'task_ids'");
      return false;
    }
    auto array_ptr = ros_message->task_ids.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: priorities
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->priorities.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->priorities);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->priorities, size)) {
      fprintf(stderr, "failed to create array for field 'priorities'");
      return false;
    }
    auto array_ptr = ros_message->priorities.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: on_critical_path
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->on_critical_path.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->on_critical_path);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->on_critical_path, size)) {
      fprintf(stderr, "failed to create array for field 'on_critical_path'");
      return false;
    }
    auto array_ptr = ros_message->on_critical_path.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: slack_times
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->slack_times.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->slack_times);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->slack_times, size)) {
      fprintf(stderr, "failed to create array for field 'slack_times'");
      return false;
    }
    auto array_ptr = ros_message->slack_times.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t get_serialized_size_dec_control__msg__TaskPriority(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TaskPriority__ros_msg_type * ros_message = static_cast<const _TaskPriority__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name task_ids
  {
    size_t array_size = ros_message->task_ids.size;
    auto array_ptr = ros_message->task_ids.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name priorities
  {
    size_t array_size = ros_message->priorities.size;
    auto array_ptr = ros_message->priorities.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name on_critical_path
  {
    size_t array_size = ros_message->on_critical_path.size;
    auto array_ptr = ros_message->on_critical_path.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name slack_times
  {
    size_t array_size = ros_message->slack_times.size;
    auto array_ptr = ros_message->slack_times.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TaskPriority__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dec_control__msg__TaskPriority(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t max_serialized_size_dec_control__msg__TaskPriority(
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

  // member: task_ids
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
  // member: priorities
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: on_critical_path
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: slack_times
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dec_control__msg__TaskPriority;
    is_plain =
      (
      offsetof(DataType, slack_times) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TaskPriority__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dec_control__msg__TaskPriority(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TaskPriority = {
  "dec_control::msg",
  "TaskPriority",
  _TaskPriority__cdr_serialize,
  _TaskPriority__cdr_deserialize,
  _TaskPriority__get_serialized_size,
  _TaskPriority__max_serialized_size
};

static rosidl_message_type_support_t _TaskPriority__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TaskPriority,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, msg, TaskPriority)() {
  return &_TaskPriority__type_support;
}

#if defined(__cplusplus)
}
#endif
