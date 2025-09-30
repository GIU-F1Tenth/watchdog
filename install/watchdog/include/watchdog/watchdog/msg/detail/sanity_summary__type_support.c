// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from watchdog:msg/SanitySummary.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "watchdog/msg/detail/sanity_summary__rosidl_typesupport_introspection_c.h"
#include "watchdog/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "watchdog/msg/detail/sanity_summary__functions.h"
#include "watchdog/msg/detail/sanity_summary__struct.h"


// Include directives for member types
// Member `sensors_with_issues`
// Member `critical_issue_summary`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  watchdog__msg__SanitySummary__init(message_memory);
}

void watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_fini_function(void * message_memory)
{
  watchdog__msg__SanitySummary__fini(message_memory);
}

size_t watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__size_function__SanitySummary__sensors_with_issues(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__get_const_function__SanitySummary__sensors_with_issues(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__get_function__SanitySummary__sensors_with_issues(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__fetch_function__SanitySummary__sensors_with_issues(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__get_const_function__SanitySummary__sensors_with_issues(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__assign_function__SanitySummary__sensors_with_issues(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__get_function__SanitySummary__sensors_with_issues(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__resize_function__SanitySummary__sensors_with_issues(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_member_array[8] = {
  {
    "system_healthy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, system_healthy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_critical_issues",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, has_critical_issues),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_errors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, has_errors),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_warnings",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, has_warnings),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sensors_with_issues",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, sensors_with_issues),  // bytes offset in struct
    NULL,  // default value
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__size_function__SanitySummary__sensors_with_issues,  // size() function pointer
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__get_const_function__SanitySummary__sensors_with_issues,  // get_const(index) function pointer
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__get_function__SanitySummary__sensors_with_issues,  // get(index) function pointer
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__fetch_function__SanitySummary__sensors_with_issues,  // fetch(index, &value) function pointer
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__assign_function__SanitySummary__sensors_with_issues,  // assign(index, value) function pointer
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__resize_function__SanitySummary__sensors_with_issues  // resize(index) function pointer
  },
  {
    "max_severity_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, max_severity_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "critical_issue_summary",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanitySummary, critical_issue_summary),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_members = {
  "watchdog__msg",  // message namespace
  "SanitySummary",  // message name
  8,  // number of fields
  sizeof(watchdog__msg__SanitySummary),
  watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_member_array,  // message members
  watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_init_function,  // function to initialize message memory (memory has to be allocated)
  watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_type_support_handle = {
  0,
  &watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_watchdog
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, watchdog, msg, SanitySummary)() {
  watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_type_support_handle.typesupport_identifier) {
    watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &watchdog__msg__SanitySummary__rosidl_typesupport_introspection_c__SanitySummary_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
