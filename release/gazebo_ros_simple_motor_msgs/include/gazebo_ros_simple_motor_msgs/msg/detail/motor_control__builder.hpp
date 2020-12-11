// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gazebo_ros_simple_motor_msgs:msg/MotorControl.idl
// generated code does not contain a copyright notice

#ifndef GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__BUILDER_HPP_
#define GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__BUILDER_HPP_

#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace gazebo_ros_simple_motor_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorControl_rpm
{
public:
  explicit Init_MotorControl_rpm(::gazebo_ros_simple_motor_msgs::msg::MotorControl & msg)
  : msg_(msg)
  {}
  ::gazebo_ros_simple_motor_msgs::msg::MotorControl rpm(::gazebo_ros_simple_motor_msgs::msg::MotorControl::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gazebo_ros_simple_motor_msgs::msg::MotorControl msg_;
};

class Init_MotorControl_angle_radians
{
public:
  explicit Init_MotorControl_angle_radians(::gazebo_ros_simple_motor_msgs::msg::MotorControl & msg)
  : msg_(msg)
  {}
  Init_MotorControl_rpm angle_radians(::gazebo_ros_simple_motor_msgs::msg::MotorControl::_angle_radians_type arg)
  {
    msg_.angle_radians = std::move(arg);
    return Init_MotorControl_rpm(msg_);
  }

private:
  ::gazebo_ros_simple_motor_msgs::msg::MotorControl msg_;
};

class Init_MotorControl_mode
{
public:
  Init_MotorControl_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControl_angle_radians mode(::gazebo_ros_simple_motor_msgs::msg::MotorControl::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorControl_angle_radians(msg_);
  }

private:
  ::gazebo_ros_simple_motor_msgs::msg::MotorControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::gazebo_ros_simple_motor_msgs::msg::MotorControl>()
{
  return gazebo_ros_simple_motor_msgs::msg::builder::Init_MotorControl_mode();
}

}  // namespace gazebo_ros_simple_motor_msgs

#endif  // GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__BUILDER_HPP_
