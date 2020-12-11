// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gazebo_ros_simple_motor_msgs:msg/MotorControl.idl
// generated code does not contain a copyright notice
#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
gazebo_ros_simple_motor_msgs__msg__MotorControl__init(gazebo_ros_simple_motor_msgs__msg__MotorControl * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // angle_radians
  // rpm
  return true;
}

void
gazebo_ros_simple_motor_msgs__msg__MotorControl__fini(gazebo_ros_simple_motor_msgs__msg__MotorControl * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // angle_radians
  // rpm
}

gazebo_ros_simple_motor_msgs__msg__MotorControl *
gazebo_ros_simple_motor_msgs__msg__MotorControl__create()
{
  gazebo_ros_simple_motor_msgs__msg__MotorControl * msg = (gazebo_ros_simple_motor_msgs__msg__MotorControl *)malloc(sizeof(gazebo_ros_simple_motor_msgs__msg__MotorControl));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gazebo_ros_simple_motor_msgs__msg__MotorControl));
  bool success = gazebo_ros_simple_motor_msgs__msg__MotorControl__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
gazebo_ros_simple_motor_msgs__msg__MotorControl__destroy(gazebo_ros_simple_motor_msgs__msg__MotorControl * msg)
{
  if (msg) {
    gazebo_ros_simple_motor_msgs__msg__MotorControl__fini(msg);
  }
  free(msg);
}


bool
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__init(gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  gazebo_ros_simple_motor_msgs__msg__MotorControl * data = NULL;
  if (size) {
    data = (gazebo_ros_simple_motor_msgs__msg__MotorControl *)calloc(size, sizeof(gazebo_ros_simple_motor_msgs__msg__MotorControl));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gazebo_ros_simple_motor_msgs__msg__MotorControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gazebo_ros_simple_motor_msgs__msg__MotorControl__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__fini(gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      gazebo_ros_simple_motor_msgs__msg__MotorControl__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence *
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__create(size_t size)
{
  gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence * array = (gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence *)malloc(sizeof(gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__destroy(gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence * array)
{
  if (array) {
    gazebo_ros_simple_motor_msgs__msg__MotorControl__Sequence__fini(array);
  }
  free(array);
}
