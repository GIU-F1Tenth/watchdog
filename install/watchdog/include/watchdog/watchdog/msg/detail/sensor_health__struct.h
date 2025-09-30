// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from watchdog:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__STRUCT_H_
#define WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sensor_name'
// Member 'active_anomalies'
#include "rosidl_runtime_c/string.h"
// Member 'last_update'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/SensorHealth in the package watchdog.
/**
  * SensorHealth.msg
  * Custom message for reporting overall health status of individual sensors
 */
typedef struct watchdog__msg__SensorHealth
{
  /// Name of the sensor (e.g., "lidar", "camera", "battery", "odometry")
  rosidl_runtime_c__String sensor_name;
  /// Overall health score from 0.0 (completely failed) to 1.0 (perfect health)
  double health_score;
  /// Whether the sensor data is currently considered valid/usable
  bool is_valid;
  /// List of currently active anomalies for this sensor
  rosidl_runtime_c__String__Sequence active_anomalies;
  /// Timestamp of the last health assessment
  builtin_interfaces__msg__Time last_update;
  /// Number of anomalies detected in the last assessment window
  uint32_t anomaly_count;
  /// Average confidence of recent assessments
  double average_confidence;
} watchdog__msg__SensorHealth;

// Struct for a sequence of watchdog__msg__SensorHealth.
typedef struct watchdog__msg__SensorHealth__Sequence
{
  watchdog__msg__SensorHealth * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} watchdog__msg__SensorHealth__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__STRUCT_H_
