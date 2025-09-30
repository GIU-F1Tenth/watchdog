// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from watchdog:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__TRAITS_HPP_
#define WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "watchdog/msg/detail/sensor_health__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'last_update'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace watchdog
{

namespace msg
{

inline void to_flow_style_yaml(
  const SensorHealth & msg,
  std::ostream & out)
{
  out << "{";
  // member: sensor_name
  {
    out << "sensor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_name, out);
    out << ", ";
  }

  // member: health_score
  {
    out << "health_score: ";
    rosidl_generator_traits::value_to_yaml(msg.health_score, out);
    out << ", ";
  }

  // member: is_valid
  {
    out << "is_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.is_valid, out);
    out << ", ";
  }

  // member: active_anomalies
  {
    if (msg.active_anomalies.size() == 0) {
      out << "active_anomalies: []";
    } else {
      out << "active_anomalies: [";
      size_t pending_items = msg.active_anomalies.size();
      for (auto item : msg.active_anomalies) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: last_update
  {
    out << "last_update: ";
    to_flow_style_yaml(msg.last_update, out);
    out << ", ";
  }

  // member: anomaly_count
  {
    out << "anomaly_count: ";
    rosidl_generator_traits::value_to_yaml(msg.anomaly_count, out);
    out << ", ";
  }

  // member: average_confidence
  {
    out << "average_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.average_confidence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SensorHealth & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sensor_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_name, out);
    out << "\n";
  }

  // member: health_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "health_score: ";
    rosidl_generator_traits::value_to_yaml(msg.health_score, out);
    out << "\n";
  }

  // member: is_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.is_valid, out);
    out << "\n";
  }

  // member: active_anomalies
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_anomalies.size() == 0) {
      out << "active_anomalies: []\n";
    } else {
      out << "active_anomalies:\n";
      for (auto item : msg.active_anomalies) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: last_update
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_update:\n";
    to_block_style_yaml(msg.last_update, out, indentation + 2);
  }

  // member: anomaly_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "anomaly_count: ";
    rosidl_generator_traits::value_to_yaml(msg.anomaly_count, out);
    out << "\n";
  }

  // member: average_confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "average_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.average_confidence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SensorHealth & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace watchdog

namespace rosidl_generator_traits
{

[[deprecated("use watchdog::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const watchdog::msg::SensorHealth & msg,
  std::ostream & out, size_t indentation = 0)
{
  watchdog::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use watchdog::msg::to_yaml() instead")]]
inline std::string to_yaml(const watchdog::msg::SensorHealth & msg)
{
  return watchdog::msg::to_yaml(msg);
}

template<>
inline const char * data_type<watchdog::msg::SensorHealth>()
{
  return "watchdog::msg::SensorHealth";
}

template<>
inline const char * name<watchdog::msg::SensorHealth>()
{
  return "watchdog/msg/SensorHealth";
}

template<>
struct has_fixed_size<watchdog::msg::SensorHealth>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<watchdog::msg::SensorHealth>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<watchdog::msg::SensorHealth>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__TRAITS_HPP_
