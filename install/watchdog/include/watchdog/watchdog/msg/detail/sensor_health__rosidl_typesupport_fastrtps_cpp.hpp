// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from watchdog:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "watchdog/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "watchdog/msg/detail/sensor_health__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace watchdog
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_watchdog
cdr_serialize(
  const watchdog::msg::SensorHealth & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_watchdog
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  watchdog::msg::SensorHealth & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_watchdog
get_serialized_size(
  const watchdog::msg::SensorHealth & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_watchdog
max_serialized_size_SensorHealth(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace watchdog

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_watchdog
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, watchdog, msg, SensorHealth)();

#ifdef __cplusplus
}
#endif

#endif  // WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
