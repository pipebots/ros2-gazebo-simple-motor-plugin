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
    <plugin name="simple_motors" filename="libgazebo_ros_simple_motors.so">
      <ros>
        <!-- Add a ROS namespace.  No code needed for this to work.  See
        https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_ros/include/gazebo_ros/node.hpp
        for details of other options.
        -->
        <namespace>/test</namespace>
      </ros>
      <!-- Maximum change in rpm per update. -->
      <max_change_rpm>2.0</max_change_rpm>
      <!-- Maximum rpm. -->
      <max_rpm>240.0</max_rpm>
      <!-- Motor shaft name -->
      <motor_shaft_name>motor_shaft_joint</motor_shaft_name>
      <!-- Update rate in Hz -->
      <update_rate>10.0</update_rate>
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
