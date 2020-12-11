// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from gazebo_ros_simple_motor_msgs:msg/MotorControl.idl
// generated code does not contain a copyright notice

#ifndef GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "gazebo_ros_simple_motor_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace gazebo_ros_simple_motor_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gazebo_ros_simple_motor_msgs
cdr_serialize(
  const gazebo_ros_simple_motor_msgs::msg::MotorControl & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gazebo_ros_simple_motor_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  gazebo_ros_simple_motor_msgs::msg::MotorControl & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gazebo_ros_simple_motor_msgs
get_serialized_size(
  const gazebo_ros_simple_motor_msgs::msg::MotorControl & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gazebo_ros_simple_motor_msgs
max_serialized_size_MotorControl(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace gazebo_ros_simple_motor_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_gazebo_ros_simple_motor_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, gazebo_ros_simple_motor_msgs, msg, MotorControl)();

#ifdef __cplusplus
}
#endif

#endif  // GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
