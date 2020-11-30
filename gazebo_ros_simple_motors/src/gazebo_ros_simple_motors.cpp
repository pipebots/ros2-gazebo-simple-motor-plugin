#include "gazebo_ros_simple_motors/gazebo_ros_simple_motors.hpp"

#include <gazebo_ros/node.hpp>
#include <sdf/sdf.hh>

#include "rclcpp/rclcpp.hpp"

#include "gazebo_ros_simple_motors/msg/motor_control.hpp"


namespace gazebo
{

class GazeboRosSimpleMotorsPrivate
{
public:
  /// Callback when a motors command is received.
  /// \param[in] _msg Motors command message.
  void OnCmdMotors(const gazebo_ros_simple_motors::msg::MotorControl::SharedPtr _msg);

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<gazebo_ros_simple_motors::msg::MotorControl>::SharedPtr cmd_motors_;

};

void GazeboRosSimpleMotorsPrivate::OnCmdMotors(const gazebo_ros_simple_motors::msg::MotorControl::SharedPtr _msg)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Received: motor %d, rpm %f",
    msg_->motor, msg_->rpm);
}



/*****************************************************************************/

GazeboRosSimpleMotors::GazeboRosSimpleMotors()
: impl_(std::make_unique<GazeboRosSimpleMotorsPrivate>())
{
}

GazeboRosSimpleMotors::~GazeboRosSimpleMotors()
{
}

void GazeboRosSimpleMotors::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles.  USed by subscribers and publishers etc.
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<gazebo_ros_simple_motors::msg::MotorControl>(
    "cmd_motors", qos.get_subscription_qos("cmd_motors", rclcpp::QoS(1)),
    std::bind(&GazeboRosSimpleMotorsPrivate::OnCmdMotors, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_motors_->get_topic_name());
}

void GazeboRosSimpleMotors::Reset()
{
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosSimpleMotors)

}  // namespace gazebo
