// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gazebo_ros_simple_motor_msgs:msg/MotorControl.idl
// generated code does not contain a copyright notice

#ifndef GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__STRUCT_HPP_
#define GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__gazebo_ros_simple_motor_msgs__msg__MotorControl __attribute__((deprecated))
#else
# define DEPRECATED__gazebo_ros_simple_motor_msgs__msg__MotorControl __declspec(deprecated)
#endif

namespace gazebo_ros_simple_motor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorControl_
{
  using Type = MotorControl_<ContainerAllocator>;

  explicit MotorControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->angle_radians = 0.0;
      this->rpm = 0.0;
    }
  }

  explicit MotorControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->angle_radians = 0.0;
      this->rpm = 0.0;
    }
  }

  // field types and members
  using _mode_type =
    int8_t;
  _mode_type mode;
  using _angle_radians_type =
    double;
  _angle_radians_type angle_radians;
  using _rpm_type =
    double;
  _rpm_type rpm;

  // setters for named parameter idiom
  Type & set__mode(
    const int8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__angle_radians(
    const double & _arg)
  {
    this->angle_radians = _arg;
    return *this;
  }
  Type & set__rpm(
    const double & _arg)
  {
    this->rpm = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t MODE_ABSOLUTE =
    0;
  static constexpr int8_t MODE_RELATIVE =
    1;
  static constexpr int8_t MODE_SPEED =
    2;

  // pointer types
  using RawPtr =
    gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gazebo_ros_simple_motor_msgs__msg__MotorControl
    std::shared_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gazebo_ros_simple_motor_msgs__msg__MotorControl
    std::shared_ptr<gazebo_ros_simple_motor_msgs::msg::MotorControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorControl_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->angle_radians != other.angle_radians) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorControl_

// alias to use template instance with default allocator
using MotorControl =
  gazebo_ros_simple_motor_msgs::msg::MotorControl_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t MotorControl_<ContainerAllocator>::MODE_ABSOLUTE;
template<typename ContainerAllocator>
constexpr int8_t MotorControl_<ContainerAllocator>::MODE_RELATIVE;
template<typename ContainerAllocator>
constexpr int8_t MotorControl_<ContainerAllocator>::MODE_SPEED;

}  // namespace msg

}  // namespace gazebo_ros_simple_motor_msgs

#endif  // GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__STRUCT_HPP_
