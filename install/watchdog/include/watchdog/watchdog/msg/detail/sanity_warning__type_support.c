// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from watchdog:msg/SanityWarning.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "watchdog/msg/detail/sanity_warning__rosidl_typesupport_introspection_c.h"
#include "watchdog/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "watchdog/msg/detail/sanity_warning__functions.h"
#include "watchdog/msg/detail/sanity_warning__struct.h"


// Include directives for member types
// Member `sensor_name`
// Member `anomaly_type`
// Member `description`
// Member `suggested_action`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  watchdog__msg__SanityWarning__init(message_memory);
}

void watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_fini_function(void * message_memory)
{
  watchdog__msg__SanityWarning__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_member_array[7] = {
  {
    "sensor_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanityWarning, sensor_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anomaly_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanityWarning, anomaly_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "severity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanityWarning, severity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanityWarning, description),  // bytes offset in struct
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
    offsetof(watchdog__msg__SanityWarning, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "suggested_action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanityWarning, suggested_action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(watchdog__msg__SanityWarning, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_members = {
  "watchdog__msg",  // message namespace
  "SanityWarning",  // message name
  7,  // number of fields
  sizeof(watchdog__msg__SanityWarning),
  watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_member_array,  // message members
  watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_init_function,  // function to initialize message memory (memory has to be allocated)
  watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_type_support_handle = {
  0,
  &watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_watchdog
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, watchdog, msg, SanityWarning)() {
  watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_type_support_handle.typesupport_identifier) {
    watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &watchdog__msg__SanityWarning__rosidl_typesupport_introspection_c__SanityWarning_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
