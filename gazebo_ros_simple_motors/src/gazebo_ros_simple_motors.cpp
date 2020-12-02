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
#include <string>


namespace gazebo
{

class GazeboRosSimpleMotorsPrivate
{
  public:
    GazeboRosSimpleMotorsPrivate() :
      max_acceleration_(0.0),
      max_torque_(0.0),
      publish_tf_(false),
      update_rate_hz_(10) { }
    rclcpp::Logger GetLogger();
    bool SetupROSNode(sdf::ElementPtr sdf);
    bool SetupMotors(physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    void OnCmdMotors(const gazebo_ros_simple_motors_msgs::msg::MotorControl::SharedPtr msg);
    bool SetupFromSDF(sdf::ElementPtr sdf);
    void SetVelocity(const double &rpm);

    /// Pointer to model.
    gazebo::physics::ModelPtr model_;
    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;
    /// Subscriber to command velocities
    rclcpp::Subscription<gazebo_ros_simple_motors_msgs::msg::MotorControl>::SharedPtr cmd_motors_;
    /// Pointer to the joint.
    physics::JointPtr joint_;
    /// A PID controller for the joint.
    common::PID pid_;
    /// Values read from SDF file.
    std::string joint_name_;
    /// Units???
    double max_acceleration_;
    /// Units???
    double max_torque_;
    /// When true, publish the transform values.
    bool publish_tf_;
    /// The update rate in Hz.
    unsigned int update_rate_hz_;
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

bool GazeboRosSimpleMotorsPrivate::SetupROSNode(sdf::ElementPtr sdf)
{
  bool success = false;
  // Initialize ROS node.  This is done early so that the logger can be used.
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles.  USed by subscribers and publishers etc.
  const gazebo_ros::QoS & qos = ros_node_->get_qos();

  cmd_motors_ =
    ros_node_->create_subscription<gazebo_ros_simple_motors_msgs::msg::MotorControl>(
    "cmd_motors", qos.get_subscription_qos("cmd_motors", rclcpp::QoS(1)),
    std::bind(&GazeboRosSimpleMotorsPrivate::OnCmdMotors, this, std::placeholders::_1));

  RCLCPP_INFO(GetLogger(), "Subscribed to [%s]", cmd_motors_->get_topic_name());

  success = true;

  return success;
}

bool GazeboRosSimpleMotorsPrivate::SetupMotors(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  bool success = SetupFromSDF(sdf);
  if (success) {
    // Save model for later use.
    model_ = model;
    // Get the name for the joint between the shaft and the body.
    auto joint_element = sdf->GetElement("motor_shaft_name");
    auto joint_name_ = joint_element->Get<std::string>();
    RCLCPP_INFO(GetLogger(), "Using joint [%s]", joint_name_);
    // Get a pointer to the joint.
    joint_ = model_->GetJoint(joint_name_);
    if (joint_) {
      std::string scoped_joint_name = joint_->GetScopedName();
      uint32_t joint_id = joint_->GetId();
      RCLCPP_INFO(
        GetLogger(), "Joint name [%s], scoped [%s] id: %d",
        joint_name_, scoped_joint_name, joint_id);
      // model_->GetJointController()->AddJoint(joint_);


      // // Setup a P-controller, with a gain of 0.1.
      // pid_ = common::PID(0.1, 0, 0);
      // // Apply the P-controller to the joint.
      // model_->GetJointController()->SetVelocityPID(joint_name_, pid_);
      // SetVelocity(1.0);
      success = true;
    } else {
      RCLCPP_ERROR(GetLogger(), "Could not get joint [%s]", joint_name_);
    }
  }
  return success;
}

bool GazeboRosSimpleMotorsPrivate::SetupFromSDF(sdf::ElementPtr sdf)
{
  bool success = false;

  auto joint_element = sdf->GetElement("motor_shaft_name");
  if (joint_element) {
    joint_name_ = joint_element->Get<std::string>();
    max_acceleration_ = sdf->Get<double>("max_acceleration", 1.0).first;
    max_torque_ = sdf->Get<double>("max_torque", 1.0).first;
    publish_tf_ = sdf->Get<bool>("publish_tf", false).first;
    update_rate_hz_ = static_cast<unsigned int>(sdf->Get<int>("update_rate", 10).first);

    RCLCPP_INFO(GetLogger(), "Using max acceleration %d", max_acceleration_);
    RCLCPP_INFO(GetLogger(), "Using max torque %d", max_torque_);
    RCLCPP_INFO(GetLogger(), "Using joint '%s'", joint_name_);
    RCLCPP_INFO(GetLogger(), "Publish transforms %d", publish_tf_);
    RCLCPP_INFO(GetLogger(), "Using update rate %d", update_rate_hz_);
    success = true;
  }

  return success;
}

void GazeboRosSimpleMotorsPrivate::SetVelocity(const double &rpm)
{
  // Convert RPM into velocity.
  // TODO Convert RPM to velocity.
  double velocity = rpm;
  // Set the joint's target velocity.
  model_->GetJointController()->SetVelocityTarget(joint_name_, velocity);
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
  unsigned int joint_count = model->GetJointCount();
  if (joint_count == 0) {
    std::cerr << "Invalid joint count, plugin not loaded\n";
  } else {
    // std::cout << "Found " << joint_count << " joints\n";
    bool success = impl_->SetupROSNode(sdf);
    if (success) {
      success = impl_->SetupMotors(model, sdf);
      if (success) {
        RCLCPP_INFO(impl_->GetLogger(), "Attached to Gazebo");
      }
    }
  }
}

void GazeboRosSimpleMotors::Reset()
{
  // TODO
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosSimpleMotors)

}  // namespace gazebo
