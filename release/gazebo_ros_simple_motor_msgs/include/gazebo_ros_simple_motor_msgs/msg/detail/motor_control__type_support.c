// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gazebo_ros_simple_motor_msgs:msg/MotorControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__rosidl_typesupport_introspection_c.h"
#include "gazebo_ros_simple_motor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__functions.h"
#include "gazebo_ros_simple_motor_msgs/msg/detail/motor_control__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MotorControl__rosidl_typesupport_introspection_c__MotorControl_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gazebo_ros_simple_motor_msgs__msg__MotorControl__init(message_memory);
}

void MotorControl__rosidl_typesupport_introspection_c__MotorControl_fini_function(void * message_memory)
{
  gazebo_ros_simple_motor_msgs__msg__MotorControl__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_member_array[3] = {
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_ros_simple_motor_msgs__msg__MotorControl, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle_radians",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_ros_simple_motor_msgs__msg__MotorControl, angle_radians),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rpm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_ros_simple_motor_msgs__msg__MotorControl, rpm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_members = {
  "gazebo_ros_simple_motor_msgs__msg",  // message namespace
  "MotorControl",  // message name
  3,  // number of fields
  sizeof(gazebo_ros_simple_motor_msgs__msg__MotorControl),
  MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_member_array,  // message members
  MotorControl__rosidl_typesupport_introspection_c__MotorControl_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorControl__rosidl_typesupport_introspection_c__MotorControl_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_type_support_handle = {
  0,
  &MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_ros_simple_motor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_ros_simple_motor_msgs, msg, MotorControl)() {
  if (!MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_type_support_handle.typesupport_identifier) {
    MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MotorControl__rosidl_typesupport_introspection_c__MotorControl_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
