// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from watchdog:msg/SanityWarning.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_WARNING__TRAITS_HPP_
#define WATCHDOG__MSG__DETAIL__SANITY_WARNING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "watchdog/msg/detail/sanity_warning__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace watchdog
{

namespace msg
{

inline void to_flow_style_yaml(
  const SanityWarning & msg,
  std::ostream & out)
{
  out << "{";
  // member: sensor_name
  {
    out << "sensor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_name, out);
    out << ", ";
  }

  // member: anomaly_type
  {
    out << "anomaly_type: ";
    rosidl_generator_traits::value_to_yaml(msg.anomaly_type, out);
    out << ", ";
  }

  // member: severity
  {
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: suggested_action
  {
    out << "suggested_action: ";
    rosidl_generator_traits::value_to_yaml(msg.suggested_action, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SanityWarning & msg,
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

  // member: anomaly_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "anomaly_type: ";
    rosidl_generator_traits::value_to_yaml(msg.anomaly_type, out);
    out << "\n";
  }

  // member: severity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: suggested_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "suggested_action: ";
    rosidl_generator_traits::value_to_yaml(msg.suggested_action, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SanityWarning & msg, bool use_flow_style = false)
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
  const watchdog::msg::SanityWarning & msg,
  std::ostream & out, size_t indentation = 0)
{
  watchdog::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use watchdog::msg::to_yaml() instead")]]
inline std::string to_yaml(const watchdog::msg::SanityWarning & msg)
{
  return watchdog::msg::to_yaml(msg);
}

template<>
inline const char * data_type<watchdog::msg::SanityWarning>()
{
  return "watchdog::msg::SanityWarning";
}

template<>
inline const char * name<watchdog::msg::SanityWarning>()
{
  return "watchdog/msg/SanityWarning";
}

template<>
struct has_fixed_size<watchdog::msg::SanityWarning>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<watchdog::msg::SanityWarning>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<watchdog::msg::SanityWarning>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WATCHDOG__MSG__DETAIL__SANITY_WARNING__TRAITS_HPP_
