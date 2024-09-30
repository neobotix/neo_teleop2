/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NeoTeleop : public rclcpp::Node
{
public:
  NeoTeleop()
  : Node("neo_teleop2_node")
  {
    // declare Parameters
    this->declare_parameter<double>("scale_linear_x", 0.4);
    this->declare_parameter<double>("scale_linear_y", 0.4);
    this->declare_parameter<double>("scale_angular_z", 0.6);
    this->declare_parameter<int>("axis_linear_x", 1);
    this->declare_parameter<int>("axis_linear_y", 0);
    this->declare_parameter<int>("axis_angular_z", 2);
    this->declare_parameter<double>("smooth_factor", 0.2);
    this->declare_parameter<int>("deadman_button", 5);
    this->declare_parameter<double>("joy_timeout", 1.);
    this->declare_parameter<double>("min_xy_vel", 0.02);
    this->declare_parameter<double>("max_xy_vel", 1.5);
    this->declare_parameter<double>("control_rate", 50.0);
    this->declare_parameter<double>("lin_acc_limit", 1.0);

    // Get Paramters
    this->get_parameter("scale_linear_x", linear_scale_x);
    this->get_parameter("scale_linear_y", linear_scale_y);
    this->get_parameter("scale_angular_z", angular_scale_z);
    this->get_parameter("axis_linear_x", axis_linear_x);
    this->get_parameter("axis_linear_y", axis_linear_y);
    this->get_parameter("axis_angular_z", axis_angular_z);
    this->get_parameter("smooth_factor", smooth_factor);
    this->get_parameter("deadman_button", deadman_button);
    this->get_parameter("joy_timeout", joy_timeout);
    this->get_parameter("min_xy_vel", min_xy_vel);
    this->get_parameter("max_xy_vel", max_xy_vel);
    this->get_parameter("control_rate", control_rate);
    this->get_parameter("lin_acc_limit", lin_acc_limit);

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      std::bind(&NeoTeleop::joy_callback, this, _1));
  }

  void send_cmd();

protected:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void applyAccelLimit(geometry_msgs::msg::Twist & cmd_vel,
  geometry_msgs::msg::Twist & last_cmd_vel);

public:
  double control_rate = 50.0; // Just for neobotix robots

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  geometry_msgs::msg::Twist cmd_vel;
  geometry_msgs::msg::Twist last_cmd_vel;

  double linear_scale_x = 0;
  double linear_scale_y = 0;
  double angular_scale_z = 0;
  double smooth_factor = 1;
  double joy_timeout = 0;
  int axis_linear_x = -1;
  int axis_linear_y = -1;
  int axis_angular_z = -1;
  int deadman_button = -1;

  rclcpp::Time last_joy_time;
  double joy_command_x = 0;
  double joy_command_y = 0;
  double joy_command_z = 0;

  double min_xy_vel = 0.0;
  double max_xy_vel = 0.0;
  double lin_acc_limit = 0.0;

  bool is_active = false;
  bool is_deadman_pressed = false;
};


void NeoTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  if (deadman_button >= 0 && deadman_button < static_cast<int>(joy->buttons.size())) {
    is_deadman_pressed = static_cast<bool>(joy->buttons[deadman_button]);
  } else {
    is_deadman_pressed = false;
  }
  if (is_deadman_pressed) {
    is_active = true;
    last_joy_time = rclcpp::Clock().now();
  }

  if (axis_linear_x >= 0 && axis_linear_x < static_cast<int>(joy->axes.size())) {
    joy_command_x = linear_scale_x * joy->axes[axis_linear_x];
  }
  if (axis_linear_y >= 0 && axis_linear_y < static_cast<int>(joy->axes.size())) {
    joy_command_y = linear_scale_y * joy->axes[axis_linear_y];
  }
  if (axis_angular_z >= 0 && axis_angular_z < static_cast<int>(joy->axes.size())) {
    joy_command_z = angular_scale_z * joy->axes[axis_angular_z];
  }
}

inline double sign(double value) {
  return (value > 0.0) ? 1.0 * value : -1.0 * value;
}

void NeoTeleop::applyAccelLimit(geometry_msgs::msg::Twist & cmd_vel, geometry_msgs::msg::Twist & last_cmd_vel)
{
  double min_possible_x_vel = last_cmd_vel.linear.x - lin_acc_limit * (1/control_rate);
  // min_possible_x_vel = (fabs(min_possible_x_vel) <= 0.02 ) ? 0.0: min_possible_x_vel;
  double max_possible_x_vel = last_cmd_vel.linear.x + lin_acc_limit * (1/control_rate);
  // max_possible_x_vel = (fabs(max_possible_x_vel) > 2.0 ) ? sign(max_possible_x_vel) * 0.8: max_possible_x_vel;

  double min_possible_y_vel = last_cmd_vel.linear.y - lin_acc_limit * (1/control_rate);
  // min_possible_y_vel = (fabs(min_possible_y_vel) <= 0.02 ) ? 0.0: min_possible_y_vel;
  double max_possible_y_vel = last_cmd_vel.linear.y + lin_acc_limit * (1/control_rate);
  // max_possible_y_vel = (fabs(max_possible_y_vel) > 2.0 ) ? sign(max_possible_y_vel) * 0.8: max_possible_y_vel;

  double min_possible_yaw_vel = last_cmd_vel.angular.z - lin_acc_limit * (1/control_rate);
  double max_possible_yaw_vel = last_cmd_vel.angular.z + lin_acc_limit * (1/control_rate);

  cmd_vel.linear.x = std::min(cmd_vel.linear.x, max_possible_x_vel);
  cmd_vel.linear.x = std::max(cmd_vel.linear.x, min_possible_x_vel);

  cmd_vel.linear.y = std::min(cmd_vel.linear.y, max_possible_y_vel);
  cmd_vel.linear.y = std::max(cmd_vel.linear.y, min_possible_y_vel);

  cmd_vel.angular.z = std::min(cmd_vel.angular.z, max_possible_yaw_vel);
  cmd_vel.angular.z = std::max(cmd_vel.angular.z, min_possible_yaw_vel);

}

void NeoTeleop::send_cmd()
{
  if (is_deadman_pressed) {
    // smooth inputs
    cmd_vel.linear.x = joy_command_x * smooth_factor + cmd_vel.linear.x * (1 - smooth_factor);
    cmd_vel.linear.y = joy_command_y * smooth_factor + cmd_vel.linear.y * (1 - smooth_factor);
    cmd_vel.angular.z = joy_command_z * smooth_factor + cmd_vel.angular.z * (1 - smooth_factor);
    applyAccelLimit(cmd_vel, last_cmd_vel);
    // publish
    vel_pub->publish(cmd_vel);
    last_cmd_vel = cmd_vel;
  } else if (is_active) {
    if ((rclcpp::Clock().now() - last_joy_time).seconds() > joy_timeout) {
      cmd_vel = geometry_msgs::msg::Twist();      // set to all zero
      last_cmd_vel = geometry_msgs::msg::Twist();      // set to all zero
      is_active = false;
    } else {
      // smooth towards zero
      cmd_vel.linear.x = cmd_vel.linear.x * (1 - smooth_factor);
      cmd_vel.linear.y = cmd_vel.linear.y * (1 - smooth_factor);
      cmd_vel.angular.z = cmd_vel.angular.z * (1 - smooth_factor);
      applyAccelLimit(cmd_vel, last_cmd_vel);
    }
    // publish
    vel_pub->publish(cmd_vel);
    last_cmd_vel = cmd_vel;
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoTeleop>();
  rclcpp::Rate loop_rate(nh->control_rate);

  while (rclcpp::ok()) {
    nh->send_cmd();

    loop_rate.sleep();

    rclcpp::spin_some(nh);
  }

  return 0;
}
