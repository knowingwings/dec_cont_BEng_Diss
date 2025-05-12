// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dec_control:msg/TaskList.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_LIST__FUNCTIONS_H_
#define DEC_CONTROL__MSG__DETAIL__TASK_LIST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dec_control/msg/rosidl_generator_c__visibility_control.h"

#include "dec_control/msg/detail/task_list__struct.h"

/// Initialize msg/TaskList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dec_control__msg__TaskList
 * )) before or use
 * dec_control__msg__TaskList__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskList__init(dec_control__msg__TaskList * msg);

/// Finalize msg/TaskList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskList__fini(dec_control__msg__TaskList * msg);

/// Create msg/TaskList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dec_control__msg__TaskList__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
dec_control__msg__TaskList *
dec_control__msg__TaskList__create();

/// Destroy msg/TaskList message.
/**
 * It calls
 * dec_control__msg__TaskList__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskList__destroy(dec_control__msg__TaskList * msg);

/// Check for msg/TaskList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskList__are_equal(const dec_control__msg__TaskList * lhs, const dec_control__msg__TaskList * rhs);

/// Copy a msg/TaskList message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskList__copy(
  const dec_control__msg__TaskList * input,
  dec_control__msg__TaskList * output);

/// Initialize array of msg/TaskList messages.
/**
 * It allocates the memory for the number of elements and calls
 * dec_control__msg__TaskList__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskList__Sequence__init(dec_control__msg__TaskList__Sequence * array, size_t size);

/// Finalize array of msg/TaskList messages.
/**
 * It calls
 * dec_control__msg__TaskList__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskList__Sequence__fini(dec_control__msg__TaskList__Sequence * array);

/// Create array of msg/TaskList messages.
/**
 * It allocates the memory for the array and calls
 * dec_control__msg__TaskList__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
dec_control__msg__TaskList__Sequence *
dec_control__msg__TaskList__Sequence__create(size_t size);

/// Destroy array of msg/TaskList messages.
/**
 * It calls
 * dec_control__msg__TaskList__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskList__Sequence__destroy(dec_control__msg__TaskList__Sequence * array);

/// Check for msg/TaskList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskList__Sequence__are_equal(const dec_control__msg__TaskList__Sequence * lhs, const dec_control__msg__TaskList__Sequence * rhs);

/// Copy an array of msg/TaskList messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskList__Sequence__copy(
  const dec_control__msg__TaskList__Sequence * input,
  dec_control__msg__TaskList__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_LIST__FUNCTIONS_H_
