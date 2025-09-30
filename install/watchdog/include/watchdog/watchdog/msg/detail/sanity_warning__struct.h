// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from watchdog:msg/SanityWarning.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_WARNING__STRUCT_H_
#define WATCHDOG__MSG__DETAIL__SANITY_WARNING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SEVERITY_INFO'.
/**
  * Constants for severity levels
 */
enum
{
  watchdog__msg__SanityWarning__SEVERITY_INFO = 1
};

/// Constant 'SEVERITY_WARNING'.
enum
{
  watchdog__msg__SanityWarning__SEVERITY_WARNING = 2
};

/// Constant 'SEVERITY_ERROR'.
enum
{
  watchdog__msg__SanityWarning__SEVERITY_ERROR = 3
};

/// Constant 'SEVERITY_CRITICAL'.
enum
{
  watchdog__msg__SanityWarning__SEVERITY_CRITICAL = 4
};

// Include directives for member types
// Member 'sensor_name'
// Member 'anomaly_type'
// Member 'description'
// Member 'suggested_action'
#include "rosidl_runtime_c/string.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/SanityWarning in the package watchdog.
/**
  * SanityWarning.msg
  * Custom message for reporting sensor data anomalies detected by the watchdog
 */
typedef struct watchdog__msg__SanityWarning
{
  /// Which sensor detected the anomaly
  rosidl_runtime_c__String sensor_name;
  /// Type of anomaly detected (e.g., "range_violation", "noise_excessive", "data_corrupted")
  rosidl_runtime_c__String anomaly_type;
  /// Severity level: 1=INFO, 2=WARNING, 3=ERROR, 4=CRITICAL
  uint8_t severity;
  /// Human-readable description of the problem
  rosidl_runtime_c__String description;
  /// When the anomaly was detected
  builtin_interfaces__msg__Time timestamp;
  /// Recommended action (e.g., "switch_to_backup", "reduce_speed", "stop_vehicle")
  rosidl_runtime_c__String suggested_action;
  /// Confidence level of the detection (0.0 to 1.0)
  double confidence;
} watchdog__msg__SanityWarning;

// Struct for a sequence of watchdog__msg__SanityWarning.
typedef struct watchdog__msg__SanityWarning__Sequence
{
  watchdog__msg__SanityWarning * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} watchdog__msg__SanityWarning__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WATCHDOG__MSG__DETAIL__SANITY_WARNING__STRUCT_H_
