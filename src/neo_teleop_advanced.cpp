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
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <neo_srvs2/srv/relay_board_set_em_stop.hpp>
#include <neo_srvs2/srv/relay_board_un_set_em_stop.hpp>
#include <neo_msgs2/msg/emergency_stop_state.hpp>

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class NeoTeleopAdvanced : public rclcpp::Node
{
public:
  NeoTeleopAdvanced()
  : Node("neo_teleop2_advanced_node")
  {
    // declare Parameters
    this->declare_parameter<double>("scale_linear", 0.4);
    this->declare_parameter<double>("scale_angular", 0.6);
    this->declare_parameter<double>("scale_linear_turbo", 0.7);
    this->declare_parameter<double>("scale_angular_turbo", 0.7);
    this->declare_parameter<int>("axis_linear_x", 1);
    this->declare_parameter<int>("axis_linear_y", 0);
    this->declare_parameter<int>("axis_angular_z", 2);
    this->declare_parameter<double>("smooth_factor", 0.2);
    this->declare_parameter<int>("deadman_button", 5);
    this->declare_parameter<int>("stop_button", 6);
    this->declare_parameter<double>("joy_timeout", 1.);
    this->declare_parameter<int>("enable_button", 6);
    this->declare_parameter<int>("enable_turbo_button", 7);

    // Get Paramters
    this->get_parameter("scale_linear", linear_scale);
    this->get_parameter("scale_angular", angular_scale);
    this->get_parameter("scale_linear_turbo", linear_scale_turbo);
    this->get_parameter("scale_angular_turbo", angular_scale_turbo);
    this->get_parameter("axis_linear_x", axis_linear_x);
    this->get_parameter("axis_linear_y", axis_linear_y);
    this->get_parameter("axis_angular_z", axis_angular_z);
    this->get_parameter("smooth_factor", smooth_factor);
    this->get_parameter("deadman_button", deadman_button);
    this->get_parameter("stop_button", stop_button);
    this->get_parameter("enable_button", enable_button);
    this->get_parameter("enable_turbo_button", enable_turbo_button);
    this->get_parameter("joy_timeout", joy_timeout);

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    feedback_pub = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 1);

    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      std::bind(&NeoTeleopAdvanced::joy_callback, this, _1));
    emstop_sub = this->create_subscription<neo_msgs2::msg::EmergencyStopState>(
      "emergency_stop_state", 1,
      std::bind(&NeoTeleopAdvanced::emstop_callback, this, _1));

    set_relay_client = this->create_client<neo_srvs2::srv::RelayBoardSetEMStop>(
      "set_EMstop");
    unset_relay_client = this->create_client<neo_srvs2::srv::RelayBoardUnSetEMStop>(
      "unset_EMstop");
  }

  void send_cmd();

protected:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void emstop_callback(const neo_msgs2::msg::EmergencyStopState::SharedPtr msg);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_pub;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<neo_msgs2::msg::EmergencyStopState>::SharedPtr emstop_sub;

  rclcpp::Client<neo_srvs2::srv::RelayBoardSetEMStop>::SharedPtr set_relay_client;
  rclcpp::Client<neo_srvs2::srv::RelayBoardUnSetEMStop>::SharedPtr unset_relay_client;
  geometry_msgs::msg::Twist cmd_vel;
  sensor_msgs::msg::JoyFeedback rumble;

  double linear_scale = 0.0;
  double angular_scale = 0.0;
  double smooth_factor = 1.0;
  double joy_timeout = 0.0;
  double linear_scale_turbo = 0.0;
  double angular_scale_turbo = 0.0;

  rclcpp::Time m_last_cmd_emstop_time;
  rclcpp::Time m_last_cmd_rumble_time;

  int axis_linear_x = -1;
  int axis_linear_y = -1;
  int axis_angular_z = -1;
  int deadman_button = -1;
  int stop_button = -1;
  int enable_button = -1;
  int enable_turbo_button = -1;

  rclcpp::Time last_joy_time;
  double joy_command_x = 0;
  double joy_command_y = 0;
  double joy_command_z = 0;

  bool is_active = false;
  bool is_deadman_pressed = false;
  bool is_turbo_enabled = false;
  bool is_software_stop = false;
  bool is_emstop = false;
  bool turbo_state_changed = false;
  bool request_check_setEM = false;
  bool request_check_unsetEM = false;

};

void NeoTeleopAdvanced::emstop_callback(const neo_msgs2::msg::EmergencyStopState::SharedPtr msg)
{
  is_emstop = msg->emergency_button_stop || msg->scanner_stop;
  if (request_check_setEM && (rclcpp::Clock().now() - m_last_cmd_emstop_time).seconds() > 1.0) {
    if (msg->software_stop) {
      RCLCPP_INFO(this->get_logger(), "Software Emergency stop was confirmed by the service");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Software Emergency stop was not confirmed by the service");
    }
    request_check_setEM = false;
  }
  if (request_check_unsetEM && (rclcpp::Clock().now() - m_last_cmd_emstop_time).seconds() > 1.0) {
    if (!msg->software_stop) {
      RCLCPP_INFO(this->get_logger(), "Software Emergency stop was unset by the service");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Software Emergency stop was not unset by the service");
    }
    request_check_unsetEM = false;
  }
}

void NeoTeleopAdvanced::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  // By default setting the scales to the normal mode
  double ang_scale = 0.0;
  double lin_scale = 0.0;

  if (static_cast<bool>(joy->buttons[enable_turbo_button])) {
    // If turbo is enabled, then set the turbo scales
    is_turbo_enabled = true;
    turbo_state_changed = true;
  }
  if (static_cast<bool>(joy->buttons[enable_button])) {
    is_turbo_enabled = false;
    turbo_state_changed = true;
  }

  if (is_turbo_enabled) {
    ang_scale = angular_scale_turbo;
    lin_scale = linear_scale_turbo;

    if (turbo_state_changed) {
      RCLCPP_INFO(
        this->get_logger(),
        "Turbo enabled, with linear_scale set to %f and angular_scale to %f",
        lin_scale,
        angular_scale);
      turbo_state_changed = false;
    }
  } else {
    ang_scale = angular_scale;
    lin_scale = linear_scale;

    if (turbo_state_changed) {
      RCLCPP_INFO(this->get_logger(),
        "Turbo disabled, linear_scale set to %f and angular_scale to %f",
        lin_scale,
        angular_scale);
      turbo_state_changed = false;
    }
  }

  if (deadman_button >= 0 && deadman_button < static_cast<int>(joy->buttons.size())) {
    is_deadman_pressed = static_cast<bool>(joy->buttons[deadman_button]);
  } else {
    is_deadman_pressed = false;
  }

  if (is_emstop && is_deadman_pressed && (rclcpp::Clock().now() - m_last_cmd_rumble_time).seconds() > 2.0) {
    rumble.type = 1;
    rumble.intensity = 0.8;
    feedback_pub->publish(rumble);
    m_last_cmd_rumble_time = rclcpp::Clock().now();
    return;
  }

  if (static_cast<bool>(joy->buttons[stop_button]) && (rclcpp::Clock().now() - m_last_cmd_emstop_time).seconds() > 2.0) {
    if (!is_software_stop) {
      RCLCPP_INFO(this->get_logger(), "Setting software EM stop");
      auto request = std::make_shared<neo_srvs2::srv::RelayBoardSetEMStop::Request>();

        while (!set_relay_client->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
          }
          RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto result = set_relay_client->async_send_request(request);
        is_software_stop = true;
        request_check_setEM = true;
      } else  {
          RCLCPP_INFO(this->get_logger(), "Unsetting software EM stop");
          auto request = std::make_shared<neo_srvs2::srv::RelayBoardUnSetEMStop::Request>();

          while (!unset_relay_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
              return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
          }

        auto result = unset_relay_client->async_send_request(request);
        is_software_stop = false;
        request_check_unsetEM = true;
      } 
    m_last_cmd_emstop_time = rclcpp::Clock().now();
  }

  if (is_deadman_pressed) {
    is_active = true;
    last_joy_time = rclcpp::Clock().now();
  }

  if (axis_linear_x >= 0 && axis_linear_x < static_cast<int>(joy->axes.size())) {
    joy_command_x = lin_scale * joy->axes[axis_linear_x];
  }
  if (axis_linear_y >= 0 && axis_linear_y < static_cast<int>(joy->axes.size())) {
    joy_command_y = lin_scale * joy->axes[axis_linear_y];
  }
  if (axis_angular_z >= 0 && axis_angular_z < static_cast<int>(joy->axes.size())) {
    joy_command_z = ang_scale * joy->axes[axis_angular_z];
  }
}

void NeoTeleopAdvanced::send_cmd()
{
  if (is_deadman_pressed) {
    // smooth inputs
    cmd_vel.linear.x = joy_command_x * smooth_factor + cmd_vel.linear.x * (1 - smooth_factor);
    cmd_vel.linear.y = joy_command_y * smooth_factor + cmd_vel.linear.y * (1 - smooth_factor);
    cmd_vel.angular.z = joy_command_z * smooth_factor + cmd_vel.angular.z * (1 - smooth_factor);

    // publish
    vel_pub->publish(cmd_vel);
  } else if (is_active) {
    if ((rclcpp::Clock().now() - last_joy_time).seconds() > joy_timeout) {
      cmd_vel = geometry_msgs::msg::Twist();      // set to all zero
      is_active = false;
    } else {
      // smooth towards zero
      cmd_vel.linear.x = cmd_vel.linear.x * (1 - smooth_factor);
      cmd_vel.linear.y = cmd_vel.linear.y * (1 - smooth_factor);
      cmd_vel.angular.z = cmd_vel.angular.z * (1 - smooth_factor);
    }
    // publish
    vel_pub->publish(cmd_vel);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoTeleopAdvanced>();
  double control_rate = 50;
  rclcpp::Rate loop_rate(control_rate);

  while (rclcpp::ok()) {
    nh->send_cmd();

    loop_rate.sleep();

    rclcpp::spin_some(nh);
  }

  return 0;
}
