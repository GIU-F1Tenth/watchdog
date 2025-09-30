// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from watchdog:msg/SanitySummary.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__BUILDER_HPP_
#define WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "watchdog/msg/detail/sanity_summary__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace watchdog
{

namespace msg
{

namespace builder
{

class Init_SanitySummary_critical_issue_summary
{
public:
  explicit Init_SanitySummary_critical_issue_summary(::watchdog::msg::SanitySummary & msg)
  : msg_(msg)
  {}
  ::watchdog::msg::SanitySummary critical_issue_summary(::watchdog::msg::SanitySummary::_critical_issue_summary_type arg)
  {
    msg_.critical_issue_summary = std::move(arg);
    return std::move(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

class Init_SanitySummary_timestamp
{
public:
  explicit Init_SanitySummary_timestamp(::watchdog::msg::SanitySummary & msg)
  : msg_(msg)
  {}
  Init_SanitySummary_critical_issue_summary timestamp(::watchdog::msg::SanitySummary::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SanitySummary_critical_issue_summary(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

class Init_SanitySummary_max_severity_level
{
public:
  explicit Init_SanitySummary_max_severity_level(::watchdog::msg::SanitySummary & msg)
  : msg_(msg)
  {}
  Init_SanitySummary_timestamp max_severity_level(::watchdog::msg::SanitySummary::_max_severity_level_type arg)
  {
    msg_.max_severity_level = std::move(arg);
    return Init_SanitySummary_timestamp(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

class Init_SanitySummary_sensors_with_issues
{
public:
  explicit Init_SanitySummary_sensors_with_issues(::watchdog::msg::SanitySummary & msg)
  : msg_(msg)
  {}
  Init_SanitySummary_max_severity_level sensors_with_issues(::watchdog::msg::SanitySummary::_sensors_with_issues_type arg)
  {
    msg_.sensors_with_issues = std::move(arg);
    return Init_SanitySummary_max_severity_level(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

class Init_SanitySummary_has_warnings
{
public:
  explicit Init_SanitySummary_has_warnings(::watchdog::msg::SanitySummary & msg)
  : msg_(msg)
  {}
  Init_SanitySummary_sensors_with_issues has_warnings(::watchdog::msg::SanitySummary::_has_warnings_type arg)
  {
    msg_.has_warnings = std::move(arg);
    return Init_SanitySummary_sensors_with_issues(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

class Init_SanitySummary_has_errors
{
public:
  explicit Init_SanitySummary_has_errors(::watchdog::msg::SanitySummary & msg)
  : msg_(msg)
  {}
  Init_SanitySummary_has_warnings has_errors(::watchdog::msg::SanitySummary::_has_errors_type arg)
  {
    msg_.has_errors = std::move(arg);
    return Init_SanitySummary_has_warnings(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

class Init_SanitySummary_has_critical_issues
{
public:
  explicit Init_SanitySummary_has_critical_issues(::watchdog::msg::SanitySummary & msg)
  : msg_(msg)
  {}
  Init_SanitySummary_has_errors has_critical_issues(::watchdog::msg::SanitySummary::_has_critical_issues_type arg)
  {
    msg_.has_critical_issues = std::move(arg);
    return Init_SanitySummary_has_errors(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

class Init_SanitySummary_system_healthy
{
public:
  Init_SanitySummary_system_healthy()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SanitySummary_has_critical_issues system_healthy(::watchdog::msg::SanitySummary::_system_healthy_type arg)
  {
    msg_.system_healthy = std::move(arg);
    return Init_SanitySummary_has_critical_issues(msg_);
  }

private:
  ::watchdog::msg::SanitySummary msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::watchdog::msg::SanitySummary>()
{
  return watchdog::msg::builder::Init_SanitySummary_system_healthy();
}

}  // namespace watchdog

#endif  // WATCHDOG__MSG__DETAIL__SANITY_SUMMARY__BUILDER_HPP_
