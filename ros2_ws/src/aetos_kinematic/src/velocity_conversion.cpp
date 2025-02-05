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

  const _Float64 _radius = 0.1;
  const _Float64 PI = 3.14159265359;

  sVelocity _Velocity;
  sPosition _CameraPosition;
  sMotorVelocity _MotorVelocity;
  sCableLength _CableLength;

  const sCableLength _InitialCableLength={0.0, 1.0, 2.0, 1.0};

  const sPosition _Pole1={0.0, 0.0, 0.0};
  const sPosition _Pole2={0.0, 1.0, 0.0};
  const sPosition _Pole3={1.0, 1.0, 0.0};
  const sPosition _Pole4={1.0, 0.0, 0.0};
  

  void updateVelocity(const aetos_msgs::msg::Velocity & msg);

  void updateLength(const aetos_msgs::msg::EncoderValues & msg);

  _Float64 getCameraPosition();

  _Float64 getMotorVelociity();

  void forwardKinematics();

  void inverseKinematics();

  void velocity_callback(const aetos_msgs::msg::Velocity & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: velocity x: '%f', y: '%f', z: '%f'", msg.vx, msg.vy, msg.vz);
  }

  void encoder_callback(const aetos_msgs::msg::EncoderValues & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: encoder angle1: '%f', angle2: '%f', angle3: '%f', angle4: '%f'", msg.angle1, msg.angle2, msg.angle3, msg.angle4);
  }

  rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr sub_velocity;
  rclcpp::Subscription<aetos_msgs::msg::EncoderValues>::SharedPtr sub_encoder;
  rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr pub_motor_velocity;
};

void VelocityConversion::updateVelocity(const aetos_msgs::msg::Velocity & msg){
  _Velocity.vx = msg.vx;
  _Velocity.vy = msg.vy;
  _Velocity.vz = msg.vz;
  
}

void VelocityConversion::updateLength(const aetos_msgs::msg::EncoderValues & msg){
  _CableLength.l1 = msg.angle1 + _InitialCableLength.l1*_radius;
  _CableLength.l2 = msg.angle2 + _InitialCableLength.l2*_radius;
  _CableLength.l3 = msg.angle3 + _InitialCableLength.l3*_radius;
  _CableLength.l4 = msg.angle4 + _InitialCableLength.l4*_radius; 
}

void VelocityConversion:: inverseKinematics(){
  Eigen::MatrixXf J(3,4);
  Eigen::VectorXf V(3);
  Eigen::VectorXf Lv(4);

  J(0,0) = (_CameraPosition.x - _Pole1.x)/_CableLength.l1;
  J(1,0) = (_CameraPosition.y - _Pole1.y)/_CableLength.l1;
  J(2,0) = (_CameraPosition.z - _Pole1.z)/_CableLength.l1;

  J(0,1) = (_CameraPosition.x - _Pole2.x)/_CableLength.l2;
  J(1,1) = (_CameraPosition.y - _Pole2.y)/_CableLength.l2;
  J(2,1) = (_CameraPosition.z - _Pole2.z)/_CableLength.l2;

  J(0,2) = (_CameraPosition.x - _Pole3.x)/_CableLength.l3;
  J(1,2) = (_CameraPosition.y - _Pole3.y)/_CableLength.l3;
  J(2,2) = (_CameraPosition.z - _Pole3.z)/_CableLength.l3;

  J(0,3) = (_CameraPosition.x - _Pole4.x)/_CableLength.l4;
  J(1,3) = (_CameraPosition.y - _Pole4.y)/_CableLength.l4;
  J(2,3) = (_CameraPosition.z - _Pole4.z)/_CableLength.l4;

  V(0) = _Velocity.vx;
  V(1) = _Velocity.vy;
  V(2) = _Velocity.vz;

  Lv = J*V;

  _MotorVelocity.w1 = Lv(0)/(2*PI*_radius);
  _MotorVelocity.w2 = Lv(1)/(2*PI*_radius);
  _MotorVelocity.w3 = Lv(2)/(2*PI*_radius);
  _MotorVelocity.w4 = Lv(3)/(2*PI*_radius);


}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityConversion>());
  rclcpp::shutdown();
  return 0;
}
