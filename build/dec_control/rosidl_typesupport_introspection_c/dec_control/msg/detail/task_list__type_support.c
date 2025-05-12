// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dec_control:msg/TaskList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dec_control/msg/detail/task_list__rosidl_typesupport_introspection_c.h"
#include "dec_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dec_control/msg/detail/task_list__functions.h"
#include "dec_control/msg/detail/task_list__struct.h"


// Include directives for member types
// Member `tasks`
#include "dec_control/msg/task.h"
// Member `tasks`
#include "dec_control/msg/detail/task__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dec_control__msg__TaskList__init(message_memory);
}

void dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_fini_function(void * message_memory)
{
  dec_control__msg__TaskList__fini(message_memory);
}

size_t dec_control__msg__TaskList__rosidl_typesupport_introspection_c__size_function__TaskList__tasks(
  const void * untyped_member)
{
  const dec_control__msg__Task__Sequence * member =
    (const dec_control__msg__Task__Sequence *)(untyped_member);
  return member->size;
}

const void * dec_control__msg__TaskList__rosidl_typesupport_introspection_c__get_const_function__TaskList__tasks(
  const void * untyped_member, size_t index)
{
  const dec_control__msg__Task__Sequence * member =
    (const dec_control__msg__Task__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dec_control__msg__TaskList__rosidl_typesupport_introspection_c__get_function__TaskList__tasks(
  void * untyped_member, size_t index)
{
  dec_control__msg__Task__Sequence * member =
    (dec_control__msg__Task__Sequence *)(untyped_member);
  return &member->data[index];
}

void dec_control__msg__TaskList__rosidl_typesupport_introspection_c__fetch_function__TaskList__tasks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dec_control__msg__Task * item =
    ((const dec_control__msg__Task *)
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__get_const_function__TaskList__tasks(untyped_member, index));
  dec_control__msg__Task * value =
    (dec_control__msg__Task *)(untyped_value);
  *value = *item;
}

void dec_control__msg__TaskList__rosidl_typesupport_introspection_c__assign_function__TaskList__tasks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dec_control__msg__Task * item =
    ((dec_control__msg__Task *)
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__get_function__TaskList__tasks(untyped_member, index));
  const dec_control__msg__Task * value =
    (const dec_control__msg__Task *)(untyped_value);
  *item = *value;
}

bool dec_control__msg__TaskList__rosidl_typesupport_introspection_c__resize_function__TaskList__tasks(
  void * untyped_member, size_t size)
{
  dec_control__msg__Task__Sequence * member =
    (dec_control__msg__Task__Sequence *)(untyped_member);
  dec_control__msg__Task__Sequence__fini(member);
  return dec_control__msg__Task__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_member_array[1] = {
  {
    "tasks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dec_control__msg__TaskList, tasks),  // bytes offset in struct
    NULL,  // default value
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__size_function__TaskList__tasks,  // size() function pointer
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__get_const_function__TaskList__tasks,  // get_const(index) function pointer
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__get_function__TaskList__tasks,  // get(index) function pointer
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__fetch_function__TaskList__tasks,  // fetch(index, &value) function pointer
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__assign_function__TaskList__tasks,  // assign(index, value) function pointer
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__resize_function__TaskList__tasks  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_members = {
  "dec_control__msg",  // message namespace
  "TaskList",  // message name
  1,  // number of fields
  sizeof(dec_control__msg__TaskList),
  dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_member_array,  // message members
  dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_init_function,  // function to initialize message memory (memory has to be allocated)
  dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_type_support_handle = {
  0,
  &dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dec_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, TaskList)() {
  dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dec_control, msg, Task)();
  if (!dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_type_support_handle.typesupport_identifier) {
    dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dec_control__msg__TaskList__rosidl_typesupport_introspection_c__TaskList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
