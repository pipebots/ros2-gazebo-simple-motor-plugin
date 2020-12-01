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

#ifndef GAZEBO_ROS_SIMPLE_MOTORS__GAZEBO_ROS_SIMPLE_MOTORS_HPP_
#define GAZEBO_ROS_SIMPLE_MOTORS__GAZEBO_ROS_SIMPLE_MOTORS_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo
{
class GazeboRosSimpleMotorsPrivate;

/**
 * @brief A simple motor plugin.
 *
 * Example Usage:
  \code{.xml}
    <-- TODO -->
    <plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <!-- Add a namespace -->
        <namespace>/test</namespace>
      </ros>
      <!-- Update rate in Hz -->
      <update_rate>50</update_rate>
      <!-- wheels -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <!-- kinematics -->
      <wheel_separation>1.25</wheel_separation>
      <wheel_diameter>0.6</wheel_diameter>
      <!-- limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <!-- input -->
      <command_topic>cmd_vel</command_topic>
      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>
    </plugin>
  \endcode
*/

class GazeboRosSimpleMotors : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosSimpleMotors();

  /// Destructor
  ~GazeboRosSimpleMotors();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosSimpleMotorsPrivate> impl_;
};

}  // namespace gazebo

#endif  // GAZEBO_ROS_SIMPLE_MOTORS__GAZEBO_ROS_SIMPLE_MOTORS_HPP_
