// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dec_control:srv/SubmitBid.idl
// generated code does not contain a copyright notice
#include "dec_control/srv/detail/submit_bid__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dec_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dec_control/srv/detail/submit_bid__struct.h"
#include "dec_control/srv/detail/submit_bid__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // resource_availability
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // resource_availability
#include "rosidl_runtime_c/string.h"  // auction_id, robot_id
#include "rosidl_runtime_c/string_functions.h"  // auction_id, robot_id

// forward declare type support functions


using _SubmitBid_Request__ros_msg_type = dec_control__srv__SubmitBid_Request;

static bool _SubmitBid_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SubmitBid_Request__ros_msg_type * ros_message = static_cast<const _SubmitBid_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: auction_id
  {
    const rosidl_runtime_c__String * str = &ros_message->auction_id;
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

  // Field name: bid_value
  {
    cdr << ros_message->bid_value;
  }

  // Field name: resource_availability
  {
    size_t size = ros_message->resource_availability.size;
    auto array_ptr = ros_message->resource_availability.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _SubmitBid_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SubmitBid_Request__ros_msg_type * ros_message = static_cast<_SubmitBid_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: auction_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->auction_id.data) {
      rosidl_runtime_c__String__init(&ros_message->auction_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->auction_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'auction_id'\n");
      return false;
    }
  }

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

  // Field name: bid_value
  {
    cdr >> ros_message->bid_value;
  }

  // Field name: resource_availability
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->resource_availability.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->resource_availability);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->resource_availability, size)) {
      fprintf(stderr, "failed to create array for field 'resource_availability'");
      return false;
    }
    auto array_ptr = ros_message->resource_availability.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t get_serialized_size_dec_control__srv__SubmitBid_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SubmitBid_Request__ros_msg_type * ros_message = static_cast<const _SubmitBid_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name auction_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->auction_id.size + 1);
  // field.name robot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_id.size + 1);
  // field.name bid_value
  {
    size_t item_size = sizeof(ros_message->bid_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name resource_availability
  {
    size_t array_size = ros_message->resource_availability.size;
    auto array_ptr = ros_message->resource_availability.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SubmitBid_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dec_control__srv__SubmitBid_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t max_serialized_size_dec_control__srv__SubmitBid_Request(
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

  // member: auction_id
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
  // member: bid_value
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: resource_availability
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dec_control__srv__SubmitBid_Request;
    is_plain =
      (
      offsetof(DataType, resource_availability) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SubmitBid_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dec_control__srv__SubmitBid_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SubmitBid_Request = {
  "dec_control::srv",
  "SubmitBid_Request",
  _SubmitBid_Request__cdr_serialize,
  _SubmitBid_Request__cdr_deserialize,
  _SubmitBid_Request__get_serialized_size,
  _SubmitBid_Request__max_serialized_size
};

static rosidl_message_type_support_t _SubmitBid_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SubmitBid_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, SubmitBid_Request)() {
  return &_SubmitBid_Request__type_support;
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
// #include "dec_control/srv/detail/submit_bid__struct.h"
// already included above
// #include "dec_control/srv/detail/submit_bid__functions.h"
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


// forward declare type support functions


using _SubmitBid_Response__ros_msg_type = dec_control__srv__SubmitBid_Response;

static bool _SubmitBid_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SubmitBid_Response__ros_msg_type * ros_message = static_cast<const _SubmitBid_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  return true;
}

static bool _SubmitBid_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SubmitBid_Response__ros_msg_type * ros_message = static_cast<_SubmitBid_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->accepted = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t get_serialized_size_dec_control__srv__SubmitBid_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SubmitBid_Response__ros_msg_type * ros_message = static_cast<const _SubmitBid_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SubmitBid_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dec_control__srv__SubmitBid_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dec_control
size_t max_serialized_size_dec_control__srv__SubmitBid_Response(
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

  // member: accepted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dec_control__srv__SubmitBid_Response;
    is_plain =
      (
      offsetof(DataType, accepted) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SubmitBid_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dec_control__srv__SubmitBid_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SubmitBid_Response = {
  "dec_control::srv",
  "SubmitBid_Response",
  _SubmitBid_Response__cdr_serialize,
  _SubmitBid_Response__cdr_deserialize,
  _SubmitBid_Response__get_serialized_size,
  _SubmitBid_Response__max_serialized_size
};

static rosidl_message_type_support_t _SubmitBid_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SubmitBid_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, SubmitBid_Response)() {
  return &_SubmitBid_Response__type_support;
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
#include "dec_control/srv/submit_bid.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SubmitBid__callbacks = {
  "dec_control::srv",
  "SubmitBid",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, SubmitBid_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, SubmitBid_Response)(),
};

static rosidl_service_type_support_t SubmitBid__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SubmitBid__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dec_control, srv, SubmitBid)() {
  return &SubmitBid__handle;
}

#if defined(__cplusplus)
}
#endif
