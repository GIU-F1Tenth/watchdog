// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from watchdog:msg/SanityWarning.idl
// generated code does not contain a copyright notice
#include "watchdog/msg/detail/sanity_warning__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sensor_name`
// Member `anomaly_type`
// Member `description`
// Member `suggested_action`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
watchdog__msg__SanityWarning__init(watchdog__msg__SanityWarning * msg)
{
  if (!msg) {
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__init(&msg->sensor_name)) {
    watchdog__msg__SanityWarning__fini(msg);
    return false;
  }
  // anomaly_type
  if (!rosidl_runtime_c__String__init(&msg->anomaly_type)) {
    watchdog__msg__SanityWarning__fini(msg);
    return false;
  }
  // severity
  // description
  if (!rosidl_runtime_c__String__init(&msg->description)) {
    watchdog__msg__SanityWarning__fini(msg);
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    watchdog__msg__SanityWarning__fini(msg);
    return false;
  }
  // suggested_action
  if (!rosidl_runtime_c__String__init(&msg->suggested_action)) {
    watchdog__msg__SanityWarning__fini(msg);
    return false;
  }
  // confidence
  return true;
}

void
watchdog__msg__SanityWarning__fini(watchdog__msg__SanityWarning * msg)
{
  if (!msg) {
    return;
  }
  // sensor_name
  rosidl_runtime_c__String__fini(&msg->sensor_name);
  // anomaly_type
  rosidl_runtime_c__String__fini(&msg->anomaly_type);
  // severity
  // description
  rosidl_runtime_c__String__fini(&msg->description);
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // suggested_action
  rosidl_runtime_c__String__fini(&msg->suggested_action);
  // confidence
}

bool
watchdog__msg__SanityWarning__are_equal(const watchdog__msg__SanityWarning * lhs, const watchdog__msg__SanityWarning * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sensor_name), &(rhs->sensor_name)))
  {
    return false;
  }
  // anomaly_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->anomaly_type), &(rhs->anomaly_type)))
  {
    return false;
  }
  // severity
  if (lhs->severity != rhs->severity) {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->description), &(rhs->description)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // suggested_action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->suggested_action), &(rhs->suggested_action)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
watchdog__msg__SanityWarning__copy(
  const watchdog__msg__SanityWarning * input,
  watchdog__msg__SanityWarning * output)
{
  if (!input || !output) {
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__copy(
      &(input->sensor_name), &(output->sensor_name)))
  {
    return false;
  }
  // anomaly_type
  if (!rosidl_runtime_c__String__copy(
      &(input->anomaly_type), &(output->anomaly_type)))
  {
    return false;
  }
  // severity
  output->severity = input->severity;
  // description
  if (!rosidl_runtime_c__String__copy(
      &(input->description), &(output->description)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // suggested_action
  if (!rosidl_runtime_c__String__copy(
      &(input->suggested_action), &(output->suggested_action)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  return true;
}

watchdog__msg__SanityWarning *
watchdog__msg__SanityWarning__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SanityWarning * msg = (watchdog__msg__SanityWarning *)allocator.allocate(sizeof(watchdog__msg__SanityWarning), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(watchdog__msg__SanityWarning));
  bool success = watchdog__msg__SanityWarning__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
watchdog__msg__SanityWarning__destroy(watchdog__msg__SanityWarning * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    watchdog__msg__SanityWarning__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
watchdog__msg__SanityWarning__Sequence__init(watchdog__msg__SanityWarning__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SanityWarning * data = NULL;

  if (size) {
    data = (watchdog__msg__SanityWarning *)allocator.zero_allocate(size, sizeof(watchdog__msg__SanityWarning), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = watchdog__msg__SanityWarning__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        watchdog__msg__SanityWarning__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
watchdog__msg__SanityWarning__Sequence__fini(watchdog__msg__SanityWarning__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      watchdog__msg__SanityWarning__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

watchdog__msg__SanityWarning__Sequence *
watchdog__msg__SanityWarning__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SanityWarning__Sequence * array = (watchdog__msg__SanityWarning__Sequence *)allocator.allocate(sizeof(watchdog__msg__SanityWarning__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = watchdog__msg__SanityWarning__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
watchdog__msg__SanityWarning__Sequence__destroy(watchdog__msg__SanityWarning__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    watchdog__msg__SanityWarning__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
watchdog__msg__SanityWarning__Sequence__are_equal(const watchdog__msg__SanityWarning__Sequence * lhs, const watchdog__msg__SanityWarning__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!watchdog__msg__SanityWarning__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
watchdog__msg__SanityWarning__Sequence__copy(
  const watchdog__msg__SanityWarning__Sequence * input,
  watchdog__msg__SanityWarning__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(watchdog__msg__SanityWarning);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    watchdog__msg__SanityWarning * data =
      (watchdog__msg__SanityWarning *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!watchdog__msg__SanityWarning__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          watchdog__msg__SanityWarning__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!watchdog__msg__SanityWarning__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
