// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from watchdog:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__FUNCTIONS_H_
#define WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "watchdog/msg/rosidl_generator_c__visibility_control.h"

#include "watchdog/msg/detail/sensor_health__struct.h"

/// Initialize msg/SensorHealth message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * watchdog__msg__SensorHealth
 * )) before or use
 * watchdog__msg__SensorHealth__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SensorHealth__init(watchdog__msg__SensorHealth * msg);

/// Finalize msg/SensorHealth message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SensorHealth__fini(watchdog__msg__SensorHealth * msg);

/// Create msg/SensorHealth message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * watchdog__msg__SensorHealth__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
watchdog__msg__SensorHealth *
watchdog__msg__SensorHealth__create();

/// Destroy msg/SensorHealth message.
/**
 * It calls
 * watchdog__msg__SensorHealth__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SensorHealth__destroy(watchdog__msg__SensorHealth * msg);

/// Check for msg/SensorHealth message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SensorHealth__are_equal(const watchdog__msg__SensorHealth * lhs, const watchdog__msg__SensorHealth * rhs);

/// Copy a msg/SensorHealth message.
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
watchdog__msg__SensorHealth__copy(
  const watchdog__msg__SensorHealth * input,
  watchdog__msg__SensorHealth * output);

/// Initialize array of msg/SensorHealth messages.
/**
 * It allocates the memory for the number of elements and calls
 * watchdog__msg__SensorHealth__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SensorHealth__Sequence__init(watchdog__msg__SensorHealth__Sequence * array, size_t size);

/// Finalize array of msg/SensorHealth messages.
/**
 * It calls
 * watchdog__msg__SensorHealth__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SensorHealth__Sequence__fini(watchdog__msg__SensorHealth__Sequence * array);

/// Create array of msg/SensorHealth messages.
/**
 * It allocates the memory for the array and calls
 * watchdog__msg__SensorHealth__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
watchdog__msg__SensorHealth__Sequence *
watchdog__msg__SensorHealth__Sequence__create(size_t size);

/// Destroy array of msg/SensorHealth messages.
/**
 * It calls
 * watchdog__msg__SensorHealth__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
void
watchdog__msg__SensorHealth__Sequence__destroy(watchdog__msg__SensorHealth__Sequence * array);

/// Check for msg/SensorHealth message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_watchdog
bool
watchdog__msg__SensorHealth__Sequence__are_equal(const watchdog__msg__SensorHealth__Sequence * lhs, const watchdog__msg__SensorHealth__Sequence * rhs);

/// Copy an array of msg/SensorHealth messages.
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
watchdog__msg__SensorHealth__Sequence__copy(
  const watchdog__msg__SensorHealth__Sequence * input,
  watchdog__msg__SensorHealth__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__FUNCTIONS_H_
