// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from watchdog:msg/SanitySummary.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__STRUCT_HPP_
#define WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__watchdog__msg__SanitySummary __attribute__((deprecated))
#else
# define DEPRECATED__watchdog__msg__SanitySummary __declspec(deprecated)
#endif

namespace watchdog
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SanitySummary_
{
  using Type = SanitySummary_<ContainerAllocator>;

  explicit SanitySummary_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_healthy = false;
      this->has_critical_issues = false;
      this->has_errors = false;
      this->has_warnings = false;
      this->max_severity_level = 0;
      this->critical_issue_summary = "";
    }
  }

  explicit SanitySummary_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    critical_issue_summary(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_healthy = false;
      this->has_critical_issues = false;
      this->has_errors = false;
      this->has_warnings = false;
      this->max_severity_level = 0;
      this->critical_issue_summary = "";
    }
  }

  // field types and members
  using _system_healthy_type =
    bool;
  _system_healthy_type system_healthy;
  using _has_critical_issues_type =
    bool;
  _has_critical_issues_type has_critical_issues;
  using _has_errors_type =
    bool;
  _has_errors_type has_errors;
  using _has_warnings_type =
    bool;
  _has_warnings_type has_warnings;
  using _sensors_with_issues_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _sensors_with_issues_type sensors_with_issues;
  using _max_severity_level_type =
    uint8_t;
  _max_severity_level_type max_severity_level;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _critical_issue_summary_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _critical_issue_summary_type critical_issue_summary;

  // setters for named parameter idiom
  Type & set__system_healthy(
    const bool & _arg)
  {
    this->system_healthy = _arg;
    return *this;
  }
  Type & set__has_critical_issues(
    const bool & _arg)
  {
    this->has_critical_issues = _arg;
    return *this;
  }
  Type & set__has_errors(
    const bool & _arg)
  {
    this->has_errors = _arg;
    return *this;
  }
  Type & set__has_warnings(
    const bool & _arg)
  {
    this->has_warnings = _arg;
    return *this;
  }
  Type & set__sensors_with_issues(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->sensors_with_issues = _arg;
    return *this;
  }
  Type & set__max_severity_level(
    const uint8_t & _arg)
  {
    this->max_severity_level = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__critical_issue_summary(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->critical_issue_summary = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    watchdog::msg::SanitySummary_<ContainerAllocator> *;
  using ConstRawPtr =
    const watchdog::msg::SanitySummary_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<watchdog::msg::SanitySummary_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<watchdog::msg::SanitySummary_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      watchdog::msg::SanitySummary_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<watchdog::msg::SanitySummary_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      watchdog::msg::SanitySummary_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<watchdog::msg::SanitySummary_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<watchdog::msg::SanitySummary_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<watchdog::msg::SanitySummary_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__watchdog__msg__SanitySummary
    std::shared_ptr<watchdog::msg::SanitySummary_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__watchdog__msg__SanitySummary
    std::shared_ptr<watchdog::msg::SanitySummary_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SanitySummary_ & other) const
  {
    if (this->system_healthy != other.system_healthy) {
      return false;
    }
    if (this->has_critical_issues != other.has_critical_issues) {
      return false;
    }
    if (this->has_errors != other.has_errors) {
      return false;
    }
    if (this->has_warnings != other.has_warnings) {
      return false;
    }
    if (this->sensors_with_issues != other.sensors_with_issues) {
      return false;
    }
    if (this->max_severity_level != other.max_severity_level) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->critical_issue_summary != other.critical_issue_summary) {
      return false;
    }
    return true;
  }
  bool operator!=(const SanitySummary_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SanitySummary_

// alias to use template instance with default allocator
using SanitySummary =
  watchdog::msg::SanitySummary_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace watchdog

#endif  // WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__STRUCT_HPP_
