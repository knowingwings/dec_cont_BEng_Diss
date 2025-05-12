// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:srv/SubmitBid.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/srv/detail/submit_bid__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/srv/detail/submit_bid__functions.h"
#include "dec_control/srv/detail/submit_bid__struct.h"


// Include directives for member types
// Member `auction_id`
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `resource_availability`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__srv__SubmitBid_Request__init(message_memory);
}

void dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_fini_function(void * message_memory)
{
  dec_control__srv__SubmitBid_Request__fini(message_memory);
}

size_t dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__size_function__SubmitBid_Request__resource_availability(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__get_const_function__SubmitBid_Request__resource_availability(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__get_function__SubmitBid_Request__resource_availability(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__fetch_function__SubmitBid_Request__resource_availability(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__get_const_function__SubmitBid_Request__resource_availability(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__assign_function__SubmitBid_Request__resource_availability(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__get_function__SubmitBid_Request__resource_availability(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__resize_function__SubmitBid_Request__resource_availability(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_member_array[4] = {
  {
    "auction_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__srv__SubmitBid_Request, auction_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__srv__SubmitBid_Request, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bid_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__srv__SubmitBid_Request, bid_value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "resource_availability",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__srv__SubmitBid_Request, resource_availability),  // bytes offset in struct
    NULL,  // default value
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__size_function__SubmitBid_Request__resource_availability,  // size() function pointer
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__get_const_function__SubmitBid_Request__resource_availability,  // get_const(index) function pointer
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__get_function__SubmitBid_Request__resource_availability,  // get(index) function pointer
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__fetch_function__SubmitBid_Request__resource_availability,  // fetch(index, &value) function pointer
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__assign_function__SubmitBid_Request__resource_availability,  // assign(index, value) function pointer
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__resize_function__SubmitBid_Request__resource_availability  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_members = {
  "dec_control__srv",  // message namespace
  "SubmitBid_Request",  // message name
  4,  // number of fields
  sizeof(dec_control__srv__SubmitBid_Request),
  dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_member_array,  // message members
  dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_type_support_handle = {
  0,
  &dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, srv, SubmitBid_Request)() {
  if (!dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_type_support_handle.typesupport_identifier) {
    dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__srv__SubmitBid_Request__rosidl_typesupport_introspection_c__SubmitBid_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dec_control/srv/detail/submit_bid__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dec_control/srv/detail/submit_bid__functions.h"
// already included above
// #include "dec_control/srv/detail/submit_bid__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__srv__SubmitBid_Response__init(message_memory);
}

void dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_fini_function(void * message_memory)
{
  dec_control__srv__SubmitBid_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_member_array[1] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__srv__SubmitBid_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_members = {
  "dec_control__srv",  // message namespace
  "SubmitBid_Response",  // message name
  1,  // number of fields
  sizeof(dec_control__srv__SubmitBid_Response),
  dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_member_array,  // message members
  dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_type_support_handle = {
  0,
  &dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, srv, SubmitBid_Response)() {
  if (!dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_type_support_handle.typesupport_identifier) {
    dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__srv__SubmitBid_Response__rosidl_typesupport_introspection_c__SubmitBid_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dec_control/srv/detail/submit_bid__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_service_members = {
  "dec_control__srv",  // service namespace
  "SubmitBid",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_Request_message_type_support_handle,
  NULL  // response message
  // dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_Response_message_type_support_handle
};

static rosidl_service_type_support_t dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_service_type_support_handle = {
  0,
  &dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, srv, SubmitBid_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, srv, SubmitBid_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, srv, SubmitBid)() {
  if (!dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_service_type_support_handle.typesupport_identifier) {
    dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, srv, SubmitBid_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, srv, SubmitBid_Response)()->data;
  }

  return &dec_control__srv__detail__submit_bid__rosidl_typesupport_introspection_c__SubmitBid_service_type_support_handle;
}
