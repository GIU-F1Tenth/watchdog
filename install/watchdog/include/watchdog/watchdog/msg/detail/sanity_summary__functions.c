// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from watchdog:msg/SanitySummary.idl
// generated code does not contain a copyright notice
#include "watchdog/msg/detail/sanity_summary__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sensors_with_issues`
// Member `critical_issue_summary`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
watchdog__msg__SanitySummary__init(watchdog__msg__SanitySummary * msg)
{
  if (!msg) {
    return false;
  }
  // system_healthy
  // has_critical_issues
  // has_errors
  // has_warnings
  // sensors_with_issues
  if (!rosidl_runtime_c__String__Sequence__init(&msg->sensors_with_issues, 0)) {
    watchdog__msg__SanitySummary__fini(msg);
    return false;
  }
  // max_severity_level
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    watchdog__msg__SanitySummary__fini(msg);
    return false;
  }
  // critical_issue_summary
  if (!rosidl_runtime_c__String__init(&msg->critical_issue_summary)) {
    watchdog__msg__SanitySummary__fini(msg);
    return false;
  }
  return true;
}

void
watchdog__msg__SanitySummary__fini(watchdog__msg__SanitySummary * msg)
{
  if (!msg) {
    return;
  }
  // system_healthy
  // has_critical_issues
  // has_errors
  // has_warnings
  // sensors_with_issues
  rosidl_runtime_c__String__Sequence__fini(&msg->sensors_with_issues);
  // max_severity_level
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // critical_issue_summary
  rosidl_runtime_c__String__fini(&msg->critical_issue_summary);
}

bool
watchdog__msg__SanitySummary__are_equal(const watchdog__msg__SanitySummary * lhs, const watchdog__msg__SanitySummary * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // system_healthy
  if (lhs->system_healthy != rhs->system_healthy) {
    return false;
  }
  // has_critical_issues
  if (lhs->has_critical_issues != rhs->has_critical_issues) {
    return false;
  }
  // has_errors
  if (lhs->has_errors != rhs->has_errors) {
    return false;
  }
  // has_warnings
  if (lhs->has_warnings != rhs->has_warnings) {
    return false;
  }
  // sensors_with_issues
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->sensors_with_issues), &(rhs->sensors_with_issues)))
  {
    return false;
  }
  // max_severity_level
  if (lhs->max_severity_level != rhs->max_severity_level) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // critical_issue_summary
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->critical_issue_summary), &(rhs->critical_issue_summary)))
  {
    return false;
  }
  return true;
}

bool
watchdog__msg__SanitySummary__copy(
  const watchdog__msg__SanitySummary * input,
  watchdog__msg__SanitySummary * output)
{
  if (!input || !output) {
    return false;
  }
  // system_healthy
  output->system_healthy = input->system_healthy;
  // has_critical_issues
  output->has_critical_issues = input->has_critical_issues;
  // has_errors
  output->has_errors = input->has_errors;
  // has_warnings
  output->has_warnings = input->has_warnings;
  // sensors_with_issues
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->sensors_with_issues), &(output->sensors_with_issues)))
  {
    return false;
  }
  // max_severity_level
  output->max_severity_level = input->max_severity_level;
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // critical_issue_summary
  if (!rosidl_runtime_c__String__copy(
      &(input->critical_issue_summary), &(output->critical_issue_summary)))
  {
    return false;
  }
  return true;
}

watchdog__msg__SanitySummary *
watchdog__msg__SanitySummary__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SanitySummary * msg = (watchdog__msg__SanitySummary *)allocator.allocate(sizeof(watchdog__msg__SanitySummary), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(watchdog__msg__SanitySummary));
  bool success = watchdog__msg__SanitySummary__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
watchdog__msg__SanitySummary__destroy(watchdog__msg__SanitySummary * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    watchdog__msg__SanitySummary__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
watchdog__msg__SanitySummary__Sequence__init(watchdog__msg__SanitySummary__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SanitySummary * data = NULL;

  if (size) {
    data = (watchdog__msg__SanitySummary *)allocator.zero_allocate(size, sizeof(watchdog__msg__SanitySummary), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = watchdog__msg__SanitySummary__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        watchdog__msg__SanitySummary__fini(&data[i - 1]);
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
watchdog__msg__SanitySummary__Sequence__fini(watchdog__msg__SanitySummary__Sequence * array)
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
      watchdog__msg__SanitySummary__fini(&array->data[i]);
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

watchdog__msg__SanitySummary__Sequence *
watchdog__msg__SanitySummary__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SanitySummary__Sequence * array = (watchdog__msg__SanitySummary__Sequence *)allocator.allocate(sizeof(watchdog__msg__SanitySummary__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = watchdog__msg__SanitySummary__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
watchdog__msg__SanitySummary__Sequence__destroy(watchdog__msg__SanitySummary__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    watchdog__msg__SanitySummary__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
watchdog__msg__SanitySummary__Sequence__are_equal(const watchdog__msg__SanitySummary__Sequence * lhs, const watchdog__msg__SanitySummary__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!watchdog__msg__SanitySummary__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
watchdog__msg__SanitySummary__Sequence__copy(
  const watchdog__msg__SanitySummary__Sequence * input,
  watchdog__msg__SanitySummary__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(watchdog__msg__SanitySummary);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    watchdog__msg__SanitySummary * data =
      (watchdog__msg__SanitySummary *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!watchdog__msg__SanitySummary__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          watchdog__msg__SanitySummary__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!watchdog__msg__SanitySummary__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
