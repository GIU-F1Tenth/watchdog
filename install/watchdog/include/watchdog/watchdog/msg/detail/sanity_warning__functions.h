// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from watchdog:msg/SanityWarning.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_WARNING__FUNCTIONS_H_
#define WATCHDOG__MSG__DETAIL__SANITY_WARNING__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "watchdog/msg/rosidl_generator_c__visibility_control.h"

#include "watchdog/msg/detail/sanity_warning__struct.h"

/// Initialize msg/SanityWarning message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * watchdog__msg__SanityWarning
 * )) before or use
 * watchdog__msg__SanityWarning__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SanityWarning__init(watchdog__msg__SanityWarning * msg);

/// Finalize msg/SanityWarning message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SanityWarning__fini(watchdog__msg__SanityWarning * msg);

/// Create msg/SanityWarning message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * watchdog__msg__SanityWarning__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
watchdog__msg__SanityWarning *
watchdog__msg__SanityWarning__create();

/// Destroy msg/SanityWarning message.
/**
 * It calls
 * watchdog__msg__SanityWarning__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SanityWarning__destroy(watchdog__msg__SanityWarning * msg);

/// Check for msg/SanityWarning message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SanityWarning__are_equal(const watchdog__msg__SanityWarning * lhs, const watchdog__msg__SanityWarning * rhs);

/// Copy a msg/SanityWarning message.
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
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SanityWarning__copy(
  const watchdog__msg__SanityWarning * input,
  watchdog__msg__SanityWarning * output);

/// Initialize array of msg/SanityWarning messages.
/**
 * It allocates the memory for the number of elements and calls
 * watchdog__msg__SanityWarning__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SanityWarning__Sequence__init(watchdog__msg__SanityWarning__Sequence * array, size_t size);

/// Finalize array of msg/SanityWarning messages.
/**
 * It calls
 * watchdog__msg__SanityWarning__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SanityWarning__Sequence__fini(watchdog__msg__SanityWarning__Sequence * array);

/// Create array of msg/SanityWarning messages.
/**
 * It allocates the memory for the array and calls
 * watchdog__msg__SanityWarning__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
watchdog__msg__SanityWarning__Sequence *
watchdog__msg__SanityWarning__Sequence__create(size_t size);

/// Destroy array of msg/SanityWarning messages.
/**
 * It calls
 * watchdog__msg__SanityWarning__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SanityWarning__Sequence__destroy(watchdog__msg__SanityWarning__Sequence * array);

/// Check for msg/SanityWarning message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SanityWarning__Sequence__are_equal(const watchdog__msg__SanityWarning__Sequence * lhs, const watchdog__msg__SanityWarning__Sequence * rhs);

/// Copy an array of msg/SanityWarning messages.
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
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SanityWarning__Sequence__copy(
  const watchdog__msg__SanityWarning__Sequence * input,
  watchdog__msg__SanityWarning__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WATCHDOG__MSG__DETAIL__SANITY_WARNING__FUNCTIONS_H_
