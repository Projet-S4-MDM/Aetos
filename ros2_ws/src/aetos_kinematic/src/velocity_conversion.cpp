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

struct sVelocity{
  _Float64 vx;
  _Float64 vy;
  _Float64 vz;
};

struct sPosition{
  _Float64 x;
  _Float64 y;
  _Float64 z;
};

struct sMotorVelocity{
  _Float64 w1;
  _Float64 w2;
  _Float64 w3;
  _Float64 w4;
};

struct sCableLength{
  _Float64 l1;
  _Float64 l2;
  _Float64 l3;
  _Float64 l4;
};





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

  sVelocity _Velocity;
  sPosition _CameraPosition;
  sMotorVelocity _MotorVelocity;
  sCableLength _CableLength;

  const sPosition _Pole1={0.0, 0.0, 0.0};
  const sPosition _Pole2={0.0, 1.0, 0.0};
  const sPosition _Pole3={1.0, 1.0, 0.0};
  const sPosition _Pole4={1.0, 0.0, 0.0};

  const _Float64 _ArenaLength = 1.0;
  const _Float64 _ArenaHeight = 1.0;
  

  void updateVelocity(const aetos_msgs::msg::Velocity & msg);

  void updateLength(const aetos_msgs::msg::EncoderValues & msg);

  struct sPosition getCameraPosition();

  struct sMotorVelocity getMotorVelociity();

  void forwardKinematics();

  void inverseKinematics();

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

void VelocityConversion::updateVelocity(const aetos_msgs::msg::Velocity & msg){
  // velocity.vx = msg.vx;
  // velocity.vy = msg.vy;
  // velocity.vz = msg.vz;
  
}

void VelocityConversion::updateLength(const aetos_msgs::msg::EncoderValues & msg){
  // float xOrigin = 0;
  // float yOrigin = 0;
  // float zOrigin = 0;
  
  
  // sPosition.x = msg.x;
  // sPosition.y = msg.y;
  // sPosition.z = msg.z;


}

struct sPosition VelocityConversion::getCameraPosition(){
  return _CameraPosition;
}
struct sMotorVelocity VelocityConversion:: getMotorVelociity(){
  return _MotorVelocity;
}
void VelocityConversion::forwardKinematics(){
  _CameraPosition.x = (_CableLength.l1 * _CableLength.l1 + _ArenaLength * _ArenaLength - _CableLength.l4 * _CableLength.l4) / (2 * _ArenaLength);
  _CameraPosition.y = (_CableLength.l1 * _CableLength.l1 + _ArenaHeight * _ArenaHeight - _CableLength.l2 * _CableLength.l2) / (2 * _ArenaHeight);
  _CameraPosition.z = sqrt(_CableLength.l1 * _CableLength.l1 - _CameraPosition.x * _CameraPosition.x - _CameraPosition.y * _CameraPosition.y);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityConversion>());
  rclcpp::shutdown();
  return 0;
}
