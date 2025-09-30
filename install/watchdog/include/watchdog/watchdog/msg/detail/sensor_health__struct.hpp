// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from watchdog:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__STRUCT_HPP_
#define WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'last_update'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__watchdog__msg__SensorHealth __attribute__((deprecated))
#else
# define DEPRECATED__watchdog__msg__SensorHealth __declspec(deprecated)
#endif

namespace watchdog
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorHealth_
{
  using Type = SensorHealth_<ContainerAllocator>;

  explicit SensorHealth_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : last_update(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->health_score = 0.0;
      this->is_valid = false;
      this->anomaly_count = 0ul;
      this->average_confidence = 0.0;
    }
  }

  explicit SensorHealth_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sensor_name(_alloc),
    last_update(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->health_score = 0.0;
      this->is_valid = false;
      this->anomaly_count = 0ul;
      this->average_confidence = 0.0;
    }
  }

  // field types and members
  using _sensor_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sensor_name_type sensor_name;
  using _health_score_type =
    double;
  _health_score_type health_score;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;
  using _active_anomalies_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _active_anomalies_type active_anomalies;
  using _last_update_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _last_update_type last_update;
  using _anomaly_count_type =
    uint32_t;
  _anomaly_count_type anomaly_count;
  using _average_confidence_type =
    double;
  _average_confidence_type average_confidence;

  // setters for named parameter idiom
  Type & set__sensor_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sensor_name = _arg;
    return *this;
  }
  Type & set__health_score(
    const double & _arg)
  {
    this->health_score = _arg;
    return *this;
  }
  Type & set__is_valid(
    const bool & _arg)
  {
    this->is_valid = _arg;
    return *this;
  }
  Type & set__active_anomalies(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->active_anomalies = _arg;
    return *this;
  }
  Type & set__last_update(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->last_update = _arg;
    return *this;
  }
  Type & set__anomaly_count(
    const uint32_t & _arg)
  {
    this->anomaly_count = _arg;
    return *this;
  }
  Type & set__average_confidence(
    const double & _arg)
  {
    this->average_confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    watchdog::msg::SensorHealth_<ContainerAllocator> *;
  using ConstRawPtr =
    const watchdog::msg::SensorHealth_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<watchdog::msg::SensorHealth_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<watchdog::msg::SensorHealth_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      watchdog::msg::SensorHealth_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<watchdog::msg::SensorHealth_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      watchdog::msg::SensorHealth_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<watchdog::msg::SensorHealth_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<watchdog::msg::SensorHealth_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<watchdog::msg::SensorHealth_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__watchdog__msg__SensorHealth
    std::shared_ptr<watchdog::msg::SensorHealth_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__watchdog__msg__SensorHealth
    std::shared_ptr<watchdog::msg::SensorHealth_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorHealth_ & other) const
  {
    if (this->sensor_name != other.sensor_name) {
      return false;
    }
    if (this->health_score != other.health_score) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    if (this->active_anomalies != other.active_anomalies) {
      return false;
    }
    if (this->last_update != other.last_update) {
      return false;
    }
    if (this->anomaly_count != other.anomaly_count) {
      return false;
    }
    if (this->average_confidence != other.average_confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorHealth_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorHealth_

// alias to use template instance with default allocator
using SensorHealth =
  watchdog::msg::SensorHealth_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace watchdog

#endif  // WATCHDOG__MSG__DETAIL__SENSOR_HEALTH__STRUCT_HPP_
