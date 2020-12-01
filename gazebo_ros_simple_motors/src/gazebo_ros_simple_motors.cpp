/*
Copyright (c) 2020 University of Leeds.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "gazebo_ros_simple_motors/gazebo_ros_simple_motors.hpp"

#include <common/common.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/Model.hh>
#include <physics/physics.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros_simple_motors_msgs/msg/motor_control.hpp>

#include <memory>


namespace gazebo
{

class GazeboRosSimpleMotorsPrivate
{
public:
  rclcpp::Logger GetLogger();
  /// Callback when a motors command is received.
  /// \param[in] _msg Motors command message.
  void OnCmdMotors(const gazebo_ros_simple_motors_msgs::msg::MotorControl::SharedPtr msg);

  void SetupROSNode(sdf::ElementPtr sdf);
  void SetupMotors();
  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

private:
  void SetVelocity(const double &rpm);

  // A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;
  // Subscriber to command velocities
  rclcpp::Subscription<gazebo_ros_simple_motors_msgs::msg::MotorControl>::SharedPtr cmd_motors_;
  // Pointer to the joint.
  physics::JointPtr joint_;
  // A PID controller for the joint.
  common::PID pid_;
};

rclcpp::Logger GazeboRosSimpleMotorsPrivate::GetLogger()
{
  return ros_node_->get_logger();
}

void GazeboRosSimpleMotorsPrivate::OnCmdMotors(
  const gazebo_ros_simple_motors_msgs::msg::MotorControl::SharedPtr msg)
{
  RCLCPP_INFO(GetLogger(), "Received: motor %d, rpm %f", msg->motor, msg->rpm);

  SetVelocity(msg->rpm);
}

void GazeboRosSimpleMotorsPrivate::SetupROSNode(sdf::ElementPtr sdf)
{
  // Initialize ROS node
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles.  USed by subscribers and publishers etc.
  const gazebo_ros::QoS & qos = ros_node_->get_qos();

  cmd_motors_ =
    ros_node_->create_subscription<gazebo_ros_simple_motors_msgs::msg::MotorControl>(
    "cmd_motors", qos.get_subscription_qos("cmd_motors", rclcpp::QoS(1)),
    std::bind(&GazeboRosSimpleMotorsPrivate::OnCmdMotors, this, std::placeholders::_1));

  RCLCPP_INFO(
    GetLogger(), "Subscribed to [%s]",
    cmd_motors_->get_topic_name());
}

void GazeboRosSimpleMotorsPrivate::SetupMotors()
{
#if 0
  // TODO Handle multiple motors.
  physics::Joint_V all_joints = model_->GetJoints();
  for (auto const& joint: all_joints) {
    RCLCPP_INFO(GetLogger(), "Loaded joint [%s]", joint->GetName());
  }
#else
  // Get the joint
  joint_ = model_->GetJoint("motor_shaft_joint");
  // Setup a P-controller, with a gain of 0.1.
  pid_ = common::PID(0.1, 0, 0);
  // Apply the P-controller to the joint.
  model_->GetJointController()->SetVelocityPID(joint_->GetScopedName(), pid_);
  SetVelocity(0.0);
#endif
}

/// \brief Set the rotational speed of the motor.
/// \param[in] _vel New target revolutions per minute.
void GazeboRosSimpleMotorsPrivate::SetVelocity(const double &rpm)
{
  // Convert RPM into velocity.
  // TODO Make this real!
  double velocity = 1.0;
  // Set the joint's target velocity.
//   model_->GetJointController()->SetVelocityTarget(joint_->GetScopedName(), velocity);
}

/*****************************************************************************/

GazeboRosSimpleMotors::GazeboRosSimpleMotors()
: impl_(std::make_unique<GazeboRosSimpleMotorsPrivate>())
{
}

GazeboRosSimpleMotors::~GazeboRosSimpleMotors()
{
}

void GazeboRosSimpleMotors::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Safety check
  // unsigned int joint_count = model->GetJointCount();
  // if (joint_count == 0) {
  //   RCLCPP_ERROR(impl_->GetLogger(), "Invalid joint count, plugin not loaded\n");
  //   return;
  // } else {
  //   RCLCPP_INFO(impl_->GetLogger(), "Found %d joints", joint_count);
  // }

  // impl_->model_ = model;
  // impl_->SetupROSNode(sdf);
  // impl_->SetupMotors();

  RCLCPP_INFO(impl_->GetLogger(), "Attached to Gazebo");
}

void GazeboRosSimpleMotors::Reset()
{
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosSimpleMotors)

}  // namespace gazebo
