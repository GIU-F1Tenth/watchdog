// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from watchdog:msg/SanityWarning.idl
// generated code does not contain a copyright notice

#ifndef WATCHDOG__MSG__DETAIL__SANITY_WARNING__STRUCT_HPP_
#define WATCHDOG__MSG__DETAIL__SANITY_WARNING__STRUCT_HPP_

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
# define DEPRECATED__watchdog__msg__SanityWarning __attribute__((deprecated))
#else
# define DEPRECATED__watchdog__msg__SanityWarning __declspec(deprecated)
#endif

namespace watchdog
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SanityWarning_
{
  using Type = SanityWarning_<ContainerAllocator>;

  explicit SanityWarning_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->anomaly_type = "";
      this->severity = 0;
      this->description = "";
      this->suggested_action = "";
      this->confidence = 0.0;
    }
  }

  explicit SanityWarning_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sensor_name(_alloc),
    anomaly_type(_alloc),
    description(_alloc),
    timestamp(_alloc, _init),
    suggested_action(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_name = "";
      this->anomaly_type = "";
      this->severity = 0;
      this->description = "";
      this->suggested_action = "";
      this->confidence = 0.0;
    }
  }

  // field types and members
  using _sensor_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sensor_name_type sensor_name;
  using _anomaly_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _anomaly_type_type anomaly_type;
  using _severity_type =
    uint8_t;
  _severity_type severity;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _suggested_action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _suggested_action_type suggested_action;
  using _confidence_type =
    double;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__sensor_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sensor_name = _arg;
    return *this;
  }
  Type & set__anomaly_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->anomaly_type = _arg;
    return *this;
  }
  Type & set__severity(
    const uint8_t & _arg)
  {
    this->severity = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__suggested_action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->suggested_action = _arg;
    return *this;
  }
  Type & set__confidence(
    const double & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SEVERITY_INFO =
    1u;
  static constexpr uint8_t SEVERITY_WARNING =
    2u;
  static constexpr uint8_t SEVERITY_ERROR =
    3u;
  static constexpr uint8_t SEVERITY_CRITICAL =
    4u;

  // pointer types
  using RawPtr =
    watchdog::msg::SanityWarning_<ContainerAllocator> *;
  using ConstRawPtr =
    const watchdog::msg::SanityWarning_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<watchdog::msg::SanityWarning_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<watchdog::msg::SanityWarning_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      watchdog::msg::SanityWarning_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<watchdog::msg::SanityWarning_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      watchdog::msg::SanityWarning_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<watchdog::msg::SanityWarning_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<watchdog::msg::SanityWarning_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<watchdog::msg::SanityWarning_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__watchdog__msg__SanityWarning
    std::shared_ptr<watchdog::msg::SanityWarning_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__watchdog__msg__SanityWarning
    std::shared_ptr<watchdog::msg::SanityWarning_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SanityWarning_ & other) const
  {
    if (this->sensor_name != other.sensor_name) {
      return false;
    }
    if (this->anomaly_type != other.anomaly_type) {
      return false;
    }
    if (this->severity != other.severity) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->suggested_action != other.suggested_action) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const SanityWarning_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SanityWarning_

// alias to use template instance with default allocator
using SanityWarning =
  watchdog::msg::SanityWarning_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SanityWarning_<ContainerAllocator>::SEVERITY_INFO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SanityWarning_<ContainerAllocator>::SEVERITY_WARNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SanityWarning_<ContainerAllocator>::SEVERITY_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SanityWarning_<ContainerAllocator>::SEVERITY_CRITICAL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace watchdog

#endif  // WATCHDOG__MSG__DETAIL__SANITY_WARNING__STRUCT_HPP_
