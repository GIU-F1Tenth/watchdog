// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from watchdog:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__BUILDER_HPP_
#define WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "watchdog/msg/detail/sensor_health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace watchdog
{

namespace msg
{

namespace builder
{

class Init_SensorHealth_average_confidence
{
public:
  explicit Init_SensorHealth_average_confidence(::watchdog::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  ::watchdog::msg::SensorHealth average_confidence(::watchdog::msg::SensorHealth::_average_confidence_type arg)
  {
    msg_.average_confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::watchdog::msg::SensorHealth msg_;
};

class Init_SensorHealth_anomaly_count
{
public:
  explicit Init_SensorHealth_anomaly_count(::watchdog::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_average_confidence anomaly_count(::watchdog::msg::SensorHealth::_anomaly_count_type arg)
  {
    msg_.anomaly_count = std::move(arg);
    return Init_SensorHealth_average_confidence(msg_);
  }

private:
  ::watchdog::msg::SensorHealth msg_;
};

class Init_SensorHealth_last_update
{
public:
  explicit Init_SensorHealth_last_update(::watchdog::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_anomaly_count last_update(::watchdog::msg::SensorHealth::_last_update_type arg)
  {
    msg_.last_update = std::move(arg);
    return Init_SensorHealth_anomaly_count(msg_);
  }

private:
  ::watchdog::msg::SensorHealth msg_;
};

class Init_SensorHealth_active_anomalies
{
public:
  explicit Init_SensorHealth_active_anomalies(::watchdog::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_last_update active_anomalies(::watchdog::msg::SensorHealth::_active_anomalies_type arg)
  {
    msg_.active_anomalies = std::move(arg);
    return Init_SensorHealth_last_update(msg_);
  }

private:
  ::watchdog::msg::SensorHealth msg_;
};

class Init_SensorHealth_is_valid
{
public:
  explicit Init_SensorHealth_is_valid(::watchdog::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_active_anomalies is_valid(::watchdog::msg::SensorHealth::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_SensorHealth_active_anomalies(msg_);
  }

private:
  ::watchdog::msg::SensorHealth msg_;
};

class Init_SensorHealth_health_score
{
public:
  explicit Init_SensorHealth_health_score(::watchdog::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_is_valid health_score(::watchdog::msg::SensorHealth::_health_score_type arg)
  {
    msg_.health_score = std::move(arg);
    return Init_SensorHealth_is_valid(msg_);
  }

private:
  ::watchdog::msg::SensorHealth msg_;
};

class Init_SensorHealth_sensor_name
{
public:
  Init_SensorHealth_sensor_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorHealth_health_score sensor_name(::watchdog::msg::SensorHealth::_sensor_name_type arg)
  {
    msg_.sensor_name = std::move(arg);
    return Init_SensorHealth_health_score(msg_);
  }

private:
  ::watchdog::msg::SensorHealth msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::watchdog::msg::SensorHealth>()
{
  return watchdog::msg::builder::Init_SensorHealth_sensor_name();
}

}  // namespace watchdog

#endif  // WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__BUILDER_HPP_
