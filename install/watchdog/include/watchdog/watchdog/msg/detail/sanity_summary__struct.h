// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from watchdog:msg/SanitySummary.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__STRUCT_H_
#define WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sensors_with_issues'
// Member 'critical_issue_summary'
#include "rosidl_runtime_c/string.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/SanitySummary in the package watchdog.
/**
  * SanitySummary.msg
  * Quick summary message for FSM decision making
 */
typedef struct watchdog__msg__SanitySummary
{
  /// Overall system sanity status
  bool system_healthy;
  /// Are there any critical issues requiring immediate action?
  bool has_critical_issues;
  /// Are there any errors that require fallback actions?
  bool has_errors;
  /// Are there any warnings to be aware of?
  bool has_warnings;
  /// List of sensors with issues
  rosidl_runtime_c__String__Sequence sensors_with_issues;
  /// Highest severity level currently active (1-4)
  uint8_t max_severity_level;
  /// Timestamp of this summary
  builtin_interfaces__msg__Time timestamp;
  /// Brief description of most critical issue (empty if system_healthy=true)
  rosidl_runtime_c__String critical_issue_summary;
} watchdog__msg__SanitySummary;

// Struct for a sequence of watchdog__msg__SanitySummary.
typedef struct watchdog__msg__SanitySummary__Sequence
{
  watchdog__msg__SanitySummary * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} watchdog__msg__SanitySummary__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__STRUCT_H_
