// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gazebo_ros_simple_motor_msgs:msg/MotorControl.idl
// generated code does not contain a copyright notice

#ifndef GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__TRAITS_HPP_
#define GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__TRAITS_HPP_

#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gazebo_ros_simple_motor_msgs::msg::MotorControl>()
{
  return "gazebo_ros_simple_motor_msgs::msg::MotorControl";
}

template<>
inline const char * name<gazebo_ros_simple_motor_msgs::msg::MotorControl>()
{
  return "gazebo_ros_simple_motor_msgs/msg/MotorControl";
}

template<>
struct has_fixed_size<gazebo_ros_simple_motor_msgs::msg::MotorControl>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gazebo_ros_simple_motor_msgs::msg::MotorControl>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gazebo_ros_simple_motor_msgs::msg::MotorControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__TRAITS_HPP_
