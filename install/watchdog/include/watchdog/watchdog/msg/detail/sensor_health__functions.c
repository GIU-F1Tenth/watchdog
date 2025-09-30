// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from watchdog:msg/SensorHealth.idl
// generated code does not contain a copyright notice
#include "watchdog/msg/detail/sensor_health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sensor_name`
// Member `active_anomalies`
#include "rosidl_runtime_c/string_functions.h"
// Member `last_update`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
watchdog__msg__SensorHealth__init(watchdog__msg__SensorHealth * msg)
{
  if (!msg) {
    return false;
  }
  // sensor_name
  if (!rosidl_runtime_c__String__init(&msg->sensor_name)) {
    watchdog__msg__SensorHealth__fini(msg);
    return false;
  }
  // health_score
  // is_valid
  // active_anomalies
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_anomalies, 0)) {
    watchdog__msg__SensorHealth__fini(msg);
    return false;
  }
  // last_update
  if (!builtin_interfaces__msg__Time__init(&msg->last_update)) {
    watchdog__msg__SensorHealth__fini(msg);
    return false;
  }
  // anomaly_count
  // average_confidence
  return true;
}

void
watchdog__msg__SensorHealth__fini(watchdog__msg__SensorHealth * msg)
{
  if (!msg) {
    return;
  }
  // sensor_name
  rosidl_runtime_c__String__fini(&msg->sensor_name);
  // health_score
  // is_valid
  // active_anomalies
  rosidl_runtime_c__String__Sequence__fini(&msg->active_anomalies);
  // last_update
  builtin_interfaces__msg__Time__fini(&msg->last_update);
  // anomaly_count
  // average_confidence
}

bool
watchdog__msg__SensorHealth__are_equal(const watchdog__msg__SensorHealth * lhs, const watchdog__msg__SensorHealth * rhs)
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
  // health_score
  if (lhs->health_score != rhs->health_score) {
    return false;
  }
  // is_valid
  if (lhs->is_valid != rhs->is_valid) {
    return false;
  }
  // active_anomalies
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_anomalies), &(rhs->active_anomalies)))
  {
    return false;
  }
  // last_update
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->last_update), &(rhs->last_update)))
  {
    return false;
  }
  // anomaly_count
  if (lhs->anomaly_count != rhs->anomaly_count) {
    return false;
  }
  // average_confidence
  if (lhs->average_confidence != rhs->average_confidence) {
    return false;
  }
  return true;
}

bool
watchdog__msg__SensorHealth__copy(
  const watchdog__msg__SensorHealth * input,
  watchdog__msg__SensorHealth * output)
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
  // health_score
  output->health_score = input->health_score;
  // is_valid
  output->is_valid = input->is_valid;
  // active_anomalies
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_anomalies), &(output->active_anomalies)))
  {
    return false;
  }
  // last_update
  if (!builtin_interfaces__msg__Time__copy(
      &(input->last_update), &(output->last_update)))
  {
    return false;
  }
  // anomaly_count
  output->anomaly_count = input->anomaly_count;
  // average_confidence
  output->average_confidence = input->average_confidence;
  return true;
}

watchdog__msg__SensorHealth *
watchdog__msg__SensorHealth__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SensorHealth * msg = (watchdog__msg__SensorHealth *)allocator.allocate(sizeof(watchdog__msg__SensorHealth), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(watchdog__msg__SensorHealth));
  bool success = watchdog__msg__SensorHealth__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
watchdog__msg__SensorHealth__destroy(watchdog__msg__SensorHealth * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    watchdog__msg__SensorHealth__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
watchdog__msg__SensorHealth__Sequence__init(watchdog__msg__SensorHealth__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SensorHealth * data = NULL;

  if (size) {
    data = (watchdog__msg__SensorHealth *)allocator.zero_allocate(size, sizeof(watchdog__msg__SensorHealth), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = watchdog__msg__SensorHealth__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        watchdog__msg__SensorHealth__fini(&data[i - 1]);
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
watchdog__msg__SensorHealth__Sequence__fini(watchdog__msg__SensorHealth__Sequence * array)
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
      watchdog__msg__SensorHealth__fini(&array->data[i]);
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

watchdog__msg__SensorHealth__Sequence *
watchdog__msg__SensorHealth__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  watchdog__msg__SensorHealth__Sequence * array = (watchdog__msg__SensorHealth__Sequence *)allocator.allocate(sizeof(watchdog__msg__SensorHealth__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = watchdog__msg__SensorHealth__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
watchdog__msg__SensorHealth__Sequence__destroy(watchdog__msg__SensorHealth__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    watchdog__msg__SensorHealth__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
watchdog__msg__SensorHealth__Sequence__are_equal(const watchdog__msg__SensorHealth__Sequence * lhs, const watchdog__msg__SensorHealth__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!watchdog__msg__SensorHealth__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
watchdog__msg__SensorHealth__Sequence__copy(
  const watchdog__msg__SensorHealth__Sequence * input,
  watchdog__msg__SensorHealth__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(watchdog__msg__SensorHealth);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    watchdog__msg__SensorHealth * data =
      (watchdog__msg__SensorHealth *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!watchdog__msg__SensorHealth__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          watchdog__msg__SensorHealth__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!watchdog__msg__SensorHealth__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
