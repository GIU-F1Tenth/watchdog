// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from watchdog:msg/SanitySummary.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__TRAITS_HPP_
#define WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "watchdog/msg/detail/sanity_summary__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace watchdog
{

namespace msg
{

inline void to_flow_style_yaml(
  const SanitySummary & msg,
  std::ostream & out)
{
  out << "{";
  // member: system_healthy
  {
    out << "system_healthy: ";
    rosidl_generator_traits::value_to_yaml(msg.system_healthy, out);
    out << ", ";
  }

  // member: has_critical_issues
  {
    out << "has_critical_issues: ";
    rosidl_generator_traits::value_to_yaml(msg.has_critical_issues, out);
    out << ", ";
  }

  // member: has_errors
  {
    out << "has_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.has_errors, out);
    out << ", ";
  }

  // member: has_warnings
  {
    out << "has_warnings: ";
    rosidl_generator_traits::value_to_yaml(msg.has_warnings, out);
    out << ", ";
  }

  // member: sensors_with_issues
  {
    if (msg.sensors_with_issues.size() == 0) {
      out << "sensors_with_issues: []";
    } else {
      out << "sensors_with_issues: [";
      size_t pending_items = msg.sensors_with_issues.size();
      for (auto item : msg.sensors_with_issues) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: max_severity_level
  {
    out << "max_severity_level: ";
    rosidl_generator_traits::value_to_yaml(msg.max_severity_level, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: critical_issue_summary
  {
    out << "critical_issue_summary: ";
    rosidl_generator_traits::value_to_yaml(msg.critical_issue_summary, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SanitySummary & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: system_healthy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_healthy: ";
    rosidl_generator_traits::value_to_yaml(msg.system_healthy, out);
    out << "\n";
  }

  // member: has_critical_issues
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_critical_issues: ";
    rosidl_generator_traits::value_to_yaml(msg.has_critical_issues, out);
    out << "\n";
  }

  // member: has_errors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.has_errors, out);
    out << "\n";
  }

  // member: has_warnings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_warnings: ";
    rosidl_generator_traits::value_to_yaml(msg.has_warnings, out);
    out << "\n";
  }

  // member: sensors_with_issues
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sensors_with_issues.size() == 0) {
      out << "sensors_with_issues: []\n";
    } else {
      out << "sensors_with_issues:\n";
      for (auto item : msg.sensors_with_issues) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: max_severity_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_severity_level: ";
    rosidl_generator_traits::value_to_yaml(msg.max_severity_level, out);
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

  // member: critical_issue_summary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "critical_issue_summary: ";
    rosidl_generator_traits::value_to_yaml(msg.critical_issue_summary, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SanitySummary & msg, bool use_flow_style = false)
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
  const watchdog::msg::SanitySummary & msg,
  std::ostream & out, size_t indentation = 0)
{
  watchdog::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use watchdog::msg::to_yaml() instead")]]
inline std::string to_yaml(const watchdog::msg::SanitySummary & msg)
{
  return watchdog::msg::to_yaml(msg);
}

template<>
inline const char * data_type<watchdog::msg::SanitySummary>()
{
  return "watchdog::msg::SanitySummary";
}

template<>
inline const char * name<watchdog::msg::SanitySummary>()
{
  return "watchdog/msg/SanitySummary";
}

template<>
struct has_fixed_size<watchdog::msg::SanitySummary>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<watchdog::msg::SanitySummary>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<watchdog::msg::SanitySummary>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__TRAITS_HPP_
