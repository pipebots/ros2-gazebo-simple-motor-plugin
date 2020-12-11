// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from gazebo_ros_simple_motor_msgs:msg/MotorControl.idl
// generated code does not contain a copyright notice

#ifndef GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__FUNCTIONS_H_
#define GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "gazebo_ros_simple_motor_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__struct.h"

/// Initialize msg/MotorControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gazebo_ros_simple_motor_msgs__msg__MotorControl
 * )) before or use
 * gazebo_ros_simple_motor_msgs__msg__MotorControl__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
bool
gazebo_ros_simple_motor_msgs__msg__MotorControl__init(gazebo_ros_simple_motor_msgs__msg__MotorControl * msg);

/// Finalize msg/MotorControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
void
gazebo_ros_simple_motor_msgs__msg__MotorControl__fini(gazebo_ros_simple_motor_msgs__msg__MotorControl * msg);

/// Create msg/MotorControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gazebo_ros_simple_motor_msgs__msg__MotorControl__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
gazebo_ros_simple_motor_msgs__msg__MotorControl *
gazebo_ros_simple_motor_msgs__msg__MotorControl__create();

/// Destroy msg/MotorControl message.
/**
 * It calls
 * gazebo_ros_simple_motor_msgs__msg__MotorControl__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
void
gazebo_ros_simple_motor_msgs__msg__MotorControl__destroy(gazebo_ros_simple_motor_msgs__msg__MotorControl * msg);


/// Initialize array of msg/MotorControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * gazebo_ros_simple_motor_msgs__msg__MotorControl__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
bool
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__init(gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence * array, size_t size);

/// Finalize array of msg/MotorControl messages.
/**
 * It calls
 * gazebo_ros_simple_motor_msgs__msg__MotorControl__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
void
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__fini(gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence * array);

/// Create array of msg/MotorControl messages.
/**
 * It allocates the memory for the array and calls
 * gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence *
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__create(size_t size);

/// Destroy array of msg/MotorControl messages.
/**
 * It calls
 * gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gazebo_ros_simple_motor_msgs
void
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__destroy(gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // GAZEBO_ROS_SIMPLE_MOTOR_MSGS__MSG__DETAIL__MOTOR_CONTROL__FUNCTIONS_H_
