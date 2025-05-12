// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dec_control:msg/TaskAssignment.idl
// generated code does not contain a copyright notice

#ifndef DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__FUNCTIONS_H_
#define DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dec_control/msg/rosidl_generator_c__visibility_control.h"

#include "dec_control/msg/detail/task_assignment__struct.h"

/// Initialize msg/TaskAssignment message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dec_control__msg__TaskAssignment
 * )) before or use
 * dec_control__msg__TaskAssignment__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskAssignment__init(dec_control__msg__TaskAssignment * msg);

/// Finalize msg/TaskAssignment message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskAssignment__fini(dec_control__msg__TaskAssignment * msg);

/// Create msg/TaskAssignment message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dec_control__msg__TaskAssignment__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
dec_control__msg__TaskAssignment *
dec_control__msg__TaskAssignment__create();

/// Destroy msg/TaskAssignment message.
/**
 * It calls
 * dec_control__msg__TaskAssignment__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskAssignment__destroy(dec_control__msg__TaskAssignment * msg);

/// Check for msg/TaskAssignment message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskAssignment__are_equal(const dec_control__msg__TaskAssignment * lhs, const dec_control__msg__TaskAssignment * rhs);

/// Copy a msg/TaskAssignment message.
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
dec_control__msg__TaskAssignment__copy(
  const dec_control__msg__TaskAssignment * input,
  dec_control__msg__TaskAssignment * output);

/// Initialize array of msg/TaskAssignment messages.
/**
 * It allocates the memory for the number of elements and calls
 * dec_control__msg__TaskAssignment__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskAssignment__Sequence__init(dec_control__msg__TaskAssignment__Sequence * array, size_t size);

/// Finalize array of msg/TaskAssignment messages.
/**
 * It calls
 * dec_control__msg__TaskAssignment__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskAssignment__Sequence__fini(dec_control__msg__TaskAssignment__Sequence * array);

/// Create array of msg/TaskAssignment messages.
/**
 * It allocates the memory for the array and calls
 * dec_control__msg__TaskAssignment__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
dec_control__msg__TaskAssignment__Sequence *
dec_control__msg__TaskAssignment__Sequence__create(size_t size);

/// Destroy array of msg/TaskAssignment messages.
/**
 * It calls
 * dec_control__msg__TaskAssignment__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
void
dec_control__msg__TaskAssignment__Sequence__destroy(dec_control__msg__TaskAssignment__Sequence * array);

/// Check for msg/TaskAssignment message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dec_control
bool
dec_control__msg__TaskAssignment__Sequence__are_equal(const dec_control__msg__TaskAssignment__Sequence * lhs, const dec_control__msg__TaskAssignment__Sequence * rhs);

/// Copy an array of msg/TaskAssignment messages.
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
dec_control__msg__TaskAssignment__Sequence__copy(
  const dec_control__msg__TaskAssignment__Sequence * input,
  dec_control__msg__TaskAssignment__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DEC_CONTROL__MSG__DETAIL__TASK_ASSIGNMENT__FUNCTIONS_H_
