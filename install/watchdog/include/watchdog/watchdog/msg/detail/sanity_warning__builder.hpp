// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from watchdog:msg/SanityWarning.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_WARNING__BUILDER_HPP_
#define WATCHDOG__MSG__DETAIL__SANITY_WARNING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "watchdog/msg/detail/sanity_warning__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace watchdog
{

namespace msg
{

namespace builder
{

class Init_SanityWarning_confidence
{
public:
  explicit Init_SanityWarning_confidence(::watchdog::msg::SanityWarning & msg)
  : msg_(msg)
  {}
  ::watchdog::msg::SanityWarning confidence(::watchdog::msg::SanityWarning::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::watchdog::msg::SanityWarning msg_;
};

class Init_SanityWarning_suggested_action
{
public:
  explicit Init_SanityWarning_suggested_action(::watchdog::msg::SanityWarning & msg)
  : msg_(msg)
  {}
  Init_SanityWarning_confidence suggested_action(::watchdog::msg::SanityWarning::_suggested_action_type arg)
  {
    msg_.suggested_action = std::move(arg);
    return Init_SanityWarning_confidence(msg_);
  }

private:
  ::watchdog::msg::SanityWarning msg_;
};

class Init_SanityWarning_timestamp
{
public:
  explicit Init_SanityWarning_timestamp(::watchdog::msg::SanityWarning & msg)
  : msg_(msg)
  {}
  Init_SanityWarning_suggested_action timestamp(::watchdog::msg::SanityWarning::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SanityWarning_suggested_action(msg_);
  }

private:
  ::watchdog::msg::SanityWarning msg_;
};

class Init_SanityWarning_description
{
public:
  explicit Init_SanityWarning_description(::watchdog::msg::SanityWarning & msg)
  : msg_(msg)
  {}
  Init_SanityWarning_timestamp description(::watchdog::msg::SanityWarning::_description_type arg)
  {
    msg_.description = std::move(arg);
    return Init_SanityWarning_timestamp(msg_);
  }

private:
  ::watchdog::msg::SanityWarning msg_;
};

class Init_SanityWarning_severity
{
public:
  explicit Init_SanityWarning_severity(::watchdog::msg::SanityWarning & msg)
  : msg_(msg)
  {}
  Init_SanityWarning_description severity(::watchdog::msg::SanityWarning::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return Init_SanityWarning_description(msg_);
  }

private:
  ::watchdog::msg::SanityWarning msg_;
};

class Init_SanityWarning_anomaly_type
{
public:
  explicit Init_SanityWarning_anomaly_type(::watchdog::msg::SanityWarning & msg)
  : msg_(msg)
  {}
  Init_SanityWarning_severity anomaly_type(::watchdog::msg::SanityWarning::_anomaly_type_type arg)
  {
    msg_.anomaly_type = std::move(arg);
    return Init_SanityWarning_severity(msg_);
  }

private:
  ::watchdog::msg::SanityWarning msg_;
};

class Init_SanityWarning_sensor_name
{
public:
  Init_SanityWarning_sensor_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SanityWarning_anomaly_type sensor_name(::watchdog::msg::SanityWarning::_sensor_name_type arg)
  {
    msg_.sensor_name = std::move(arg);
    return Init_SanityWarning_anomaly_type(msg_);
  }

private:
  ::watchdog::msg::SanityWarning msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::watchdog::msg::SanityWarning>()
{
  return watchdog::msg::builder::Init_SanityWarning_sensor_name();
}

}  // namespace watchdog

#endif  // WATCHDOG__MSG__DETAIL__SANITY_WARNING__BUILDER_HPP_
