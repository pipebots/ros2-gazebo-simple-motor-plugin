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
      update_period_s_(0.0),
      shaft_target_rpm_(0.0),
      shaft_max_rpm_(0.0)
    {
    }
    rclcpp::Logger GetLogger();
    void Reset();
    bool SetupROSNode(sdf::ElementPtr sdf);
    bool SetupMotors(physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    void OnUpdate(const gazebo::common::UpdateInfo & _info);
    void OnCmdMotors(const gazebo_ros_simple_motors_msgs::msg::MotorControl::SharedPtr msg);
    bool SetupFromSDF(sdf::ElementPtr sdf);
    void UpdateShaftRPM();

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
    /// Joint name.
    std::string joint_name_;
    /// Units???
    double max_acceleration_;
    /// Units???
    double max_torque_;
    /// When true, publish the transform values.
    bool publish_tf_;
    /// The update period in seconds.
    double update_period_s_;
    /// Shaft revolutions per min
    double shaft_target_rpm_;
    double shaft_max_rpm_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;
    /// Last update time.
    gazebo::common::Time last_update_time_;
};

rclcpp::Logger GazeboRosSimpleMotorsPrivate::GetLogger()
{
  return ros_node_->get_logger();
}

void GazeboRosSimpleMotorsPrivate::Reset()
{
  last_update_time_ = model_->GetWorld()->SimTime();
  update_period_s_ = 0.0;
  shaft_target_rpm_ = 0.0;
  shaft_max_rpm_ = 0.0;
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

  // Listen to the update event (broadcast every simulation iteration)
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosSimpleMotorsPrivate::OnUpdate, this, std::placeholders::_1));

  success = true;

  return success;
}

bool GazeboRosSimpleMotorsPrivate::SetupMotors(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  bool success = SetupFromSDF(sdf);
  if (success) {
    // Save model for later use.
    model_ = model;
    // Set update time.
    last_update_time_ = model_->GetWorld()->SimTime();
    // Get a pointer to the joint.
    joint_ = model_->GetJoint(joint_name_);
    if (joint_) {
      success = true;
    } else {
      RCLCPP_ERROR(GetLogger(), "Could not get joint [%s]", joint_name_.c_str());
    }
  }
  return success;
}

bool GazeboRosSimpleMotorsPrivate::SetupFromSDF(sdf::ElementPtr sdf)
{
  bool success = false;
  // FIXME  These read garbage values.
  max_acceleration_ = sdf->Get<double>("max_acceleration", 1.0).first;
  max_torque_ = sdf->Get<double>("max_torque", 1.0).first;
  shaft_max_rpm_ = sdf->Get<double>("max_rpm", 100.0).first;
  // These work.
  joint_name_ = sdf->Get<std::string>("motor_shaft_name", "").first;
  publish_tf_ = sdf->Get<bool>("publish_tf", false).first;
  auto update_rate_hz = sdf->Get<double>("update_rate", 10.0).first;
  if (update_rate_hz > 0.0) {
    update_period_s_ = 1.0 / update_rate_hz;
  } else {
    update_period_s_ = 0.0;
  }

  RCLCPP_INFO(GetLogger(), "Using max acceleration %d", max_acceleration_);
  RCLCPP_INFO(GetLogger(), "Using max torque %d", max_torque_);
  RCLCPP_INFO(GetLogger(), "Using max RPM %d", shaft_max_rpm_);
  RCLCPP_INFO(GetLogger(), "Using joint [%s]", joint_name_.c_str());
  RCLCPP_INFO(GetLogger(), "Publish transforms %d", publish_tf_);
  RCLCPP_INFO(GetLogger(), "Using update period %f", update_period_s_);
  // HACK
  success = true;
  shaft_max_rpm_ = 200.0;
  shaft_target_rpm_ = 100.0;

  return success;
}

void GazeboRosSimpleMotorsPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double since_last_update_s = (_info.simTime - last_update_time_).Double();
  if (since_last_update_s > update_period_s_) {
    UpdateShaftRPM();
    last_update_time_ = _info.simTime;
  }
}

void GazeboRosSimpleMotorsPrivate::OnCmdMotors(
  const gazebo_ros_simple_motors_msgs::msg::MotorControl::SharedPtr msg)
{
  RCLCPP_INFO(GetLogger(), "Received: motor %d, rpm %f", msg->motor, msg->rpm);
  shaft_target_rpm_ = msg->rpm;
}

void GazeboRosSimpleMotorsPrivate::UpdateShaftRPM()
{
  // Velocity is in radians per second.
  // Convert using 1 rad/s = 9.55 rpm.
  const unsigned int axis = 0;
  const double increment_rpm = 0.1;
  double current_rad_s = joint_->GetVelocity(axis);
  double current_rpm = 9.55 * current_rad_s;
  double new_rpm = current_rpm;
  if (new_rpm > shaft_max_rpm_) {
    // Maximum spped.
    new_rpm = shaft_max_rpm_;
  } else if (new_rpm < (shaft_target_rpm_ - increment_rpm)) {
    // Speed up.
    new_rpm += increment_rpm;
  } else if (new_rpm > (shaft_target_rpm_ + increment_rpm)) {
    // Slow down.
    new_rpm -= increment_rpm;
  } else {
    // Reached target speed.
    new_rpm = shaft_target_rpm_;
  }
  RCLCPP_INFO(GetLogger(), "Current %f, new %f", current_rpm, new_rpm);
  double new_rad_s = new_rpm / 9.55;
  joint_->SetVelocity(axis, new_rad_s);
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
  impl_->Reset();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosSimpleMotors)

}  // namespace gazebo
