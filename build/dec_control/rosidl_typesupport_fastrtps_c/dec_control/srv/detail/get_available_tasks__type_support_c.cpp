// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dec_control:srv/GetAvailableTasks.idl
// generated code does not contain a copyright notice
#include "dec_control/srv/detail/get_available_tasks__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dec_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dec_control/srv/detail/get_available_tasks__struct.h"
#include "dec_control/srv/detail/get_available_tasks__functions.h"
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

#include "rosidl_runtime_c/string.h"  // robot_id
#include "rosidl_runtime_c/string_functions.h"  // robot_id

// forward declare type support functions


using _GetAvailableTasks_Request__ros_msg_type = dec_control__srv__GetAvailableTasks_Request;

static bool _GetAvailableTasks_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetAvailableTasks_Request__ros_msg_type * ros_message = static_cast<const _GetAvailableTasks_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_id;
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

  return true;
}

static bool _GetAvailableTasks_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetAvailableTasks_Request__ros_msg_type * ros_message = static_cast<_GetAvailableTasks_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_id.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_id'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t get_serialized_size_dec_control__srv__GetAvailableTasks_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableTasks_Request__ros_msg_type * ros_message = static_cast<const _GetAvailableTasks_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name robot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_id.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _GetAvailableTasks_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dec_control__srv__GetAvailableTasks_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t max_serialized_size_dec_control__srv__GetAvailableTasks_Request(
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

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dec_control__srv__GetAvailableTasks_Request;
    is_plain =
      (
      offsetof(DataType, robot_id) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetAvailableTasks_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dec_control__srv__GetAvailableTasks_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetAvailableTasks_Request = {
  "dec_control::srv",
  "GetAvailableTasks_Request",
  _GetAvailableTasks_Request__cdr_serialize,
  _GetAvailableTasks_Request__cdr_deserialize,
  _GetAvailableTasks_Request__get_serialized_size,
  _GetAvailableTasks_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetAvailableTasks_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetAvailableTasks_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, GetAvailableTasks_Request)() {
  return &_GetAvailableTasks_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "dec_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "dec_control/srv/detail/get_available_tasks__struct.h"
// already included above
// #include "dec_control/srv/detail/get_available_tasks__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "dec_control/msg/detail/task__functions.h"  // available_tasks

// forward declare type support functions
size_t get_serialized_size_dec_control__msg__Task(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_dec_control__msg__Task(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, msg, Task)();


using _GetAvailableTasks_Response__ros_msg_type = dec_control__srv__GetAvailableTasks_Response;

static bool _GetAvailableTasks_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetAvailableTasks_Response__ros_msg_type * ros_message = static_cast<const _GetAvailableTasks_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: available_tasks
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, dec_control, msg, Task
      )()->data);
    size_t size = ros_message->available_tasks.size;
    auto array_ptr = ros_message->available_tasks.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _GetAvailableTasks_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetAvailableTasks_Response__ros_msg_type * ros_message = static_cast<_GetAvailableTasks_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: available_tasks
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, dec_control, msg, Task
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->available_tasks.data) {
      dec_control__msg__Task__Sequence__fini(&ros_message->available_tasks);
    }
    if (!dec_control__msg__Task__Sequence__init(&ros_message->available_tasks, size)) {
      fprintf(stderr, "failed to create array for field 'available_tasks'");
      return false;
    }
    auto array_ptr = ros_message->available_tasks.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t get_serialized_size_dec_control__srv__GetAvailableTasks_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableTasks_Response__ros_msg_type * ros_message = static_cast<const _GetAvailableTasks_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name available_tasks
  {
    size_t array_size = ros_message->available_tasks.size;
    auto array_ptr = ros_message->available_tasks.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_dec_control__msg__Task(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetAvailableTasks_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dec_control__srv__GetAvailableTasks_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t max_serialized_size_dec_control__srv__GetAvailableTasks_Response(
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

  // member: available_tasks
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_dec_control__msg__Task(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dec_control__srv__GetAvailableTasks_Response;
    is_plain =
      (
      offsetof(DataType, available_tasks) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetAvailableTasks_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dec_control__srv__GetAvailableTasks_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetAvailableTasks_Response = {
  "dec_control::srv",
  "GetAvailableTasks_Response",
  _GetAvailableTasks_Response__cdr_serialize,
  _GetAvailableTasks_Response__cdr_deserialize,
  _GetAvailableTasks_Response__get_serialized_size,
  _GetAvailableTasks_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetAvailableTasks_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetAvailableTasks_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, GetAvailableTasks_Response)() {
  return &_GetAvailableTasks_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "dec_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dec_control/srv/get_available_tasks.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetAvailableTasks__callbacks = {
  "dec_control::srv",
  "GetAvailableTasks",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, GetAvailableTasks_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, GetAvailableTasks_Response)(),
};

static rosidl_service_type_support_t GetAvailableTasks__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetAvailableTasks__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, GetAvailableTasks)() {
  return &GetAvailableTasks__handle;
}

#if defined(__cplusplus)
}
#endif
