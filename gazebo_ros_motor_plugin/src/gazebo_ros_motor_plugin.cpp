#include <gazebo_ros_motor_plugin/gazebo_ros_motor_plugin.hpp>

#include <gazebo_ros/node.hpp>
#include <sdf/sdf.hh>


namespace gazebo
{

class GazeboRosMotorPrivate
{
public:


  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

};

GazeboRosMotor::GazeboRosMotor()
: impl_(std::make_unique<GazeboRosMotorPrivate>())
{
}

GazeboRosMotor::~GazeboRosMotor()
{
}

void GazeboRosMotor::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  // Needed for subscribers.
//   const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  }

void GazeboRosMotor::Reset()
{
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotor)

}  // namespace gazebo
