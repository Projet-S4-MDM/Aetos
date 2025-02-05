// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "aetos_msgs/msg/velocity.hpp"
#include "aetos_msgs/msg/encoder_values.hpp"  // Include the generated header for EncoderValues
#include "aetos_msgs/msg/motor_velocity.hpp"  // Include the generated header for MotorVelocity

using std::placeholders::_1;

class VelocityConversion : public rclcpp::Node
{
public:
  VelocityConversion()
  : Node("VelocityConversion")
  {
    sub_velocity = this->create_subscription<aetos_msgs::msg::Velocity>(
      "aetos/joy/velocity", 10, std::bind(&VelocityConversion::velocity_callback, this, _1));

    sub_encoder = this->create_subscription<aetos_msgs::msg::EncoderValues>(
      "aetos/control/encoder", 10, std::bind(&VelocityConversion::encoder_callback, this, _1));

    pub_motor_velocity = this->create_publisher<aetos_msgs::msg::MotorVelocity>("aetos/control/velocity", 10);  // Use the generated message type
  }

private:
  float vx, vy, vz; // Vitesse
  float x, y, z; // Position

  void update_velocity(const aetos_msgs::msg::Velocity & msg);

  void update_position();

  void velocity_callback(const aetos_msgs::msg::Velocity & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: velocity x: '%f', y: '%f', z: '%f'", msg.vx, msg.vy, msg.vz);
  }

  void encoder_callback(const aetos_msgs::msg::EncoderValues & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: encoder values");
  }

  rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr sub_velocity;
  rclcpp::Subscription<aetos_msgs::msg::EncoderValues>::SharedPtr sub_encoder;
  rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr pub_motor_velocity;
};

void VelocityConversion::update_velocity(const aetos_msgs::msg::Velocity & msg){
  vx = msg.vx;
  vy = msg.vy;
  vz = msg.vz;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityConversion>());
  rclcpp::shutdown();
  return 0;
}
