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
#include "aetos_msgs/msg/effector_position.hpp"  // Include the generated header for EffectorPosition

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

    pub_motor_velocity = this->create_publisher<aetos_msgs::msg::MotorVelocity>("aetos/control/velocity", 10);  

    pub_position = this->create_publisher<aetos_msgs::msg::EffectorPosition>("aetos/control/position", 10);// Use the generated message type
  }

private:

  const _Float64 PI = 3.14159265359;

  sVelocity _Velocity;
  sPosition _CameraPosition;
  sMotorVelocity _MotorVelocity;
  sCableLength _CableLength;

  //SETUP
  const _Float64 _radius = 0.05;

  const _Float64 _ArenaLength = 0.816;
  const _Float64 _ArenaWidth = 0.816;
  const _Float64 _ArenaHeight = 0.7;

  const sCableLength _InitialCableLength = {0.05, 0.78144487, 1.10399827, 0.78144487};

  const sPosition _Pole1={0.0, 0.0, 0.0};
  const sPosition _Pole2={0.0, _ArenaWidth, 0.0};
  const sPosition _Pole3={_ArenaLength, _ArenaWidth, 0.0};
  const sPosition _Pole4={_ArenaLength, 0.0, 0.0};
  

  void updateVelocity(const aetos_msgs::msg::Velocity & msg);

  void updateLength(const aetos_msgs::msg::EncoderValues & msg);

  void forwardKinematics();

  void inverseKinematics();

  void uavInBoundSecurityCheck();

  void velocity_callback(const aetos_msgs::msg::Velocity & msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: velocity vx: '%f', vy: '%f', vz: '%f'", msg.vx, msg.vy, msg.vz);
    this->updateVelocity(msg);
    this->forwardKinematics();
    this->inverseKinematics();
    this->publish_motor_velocity(_MotorVelocity);
    this->publish_position(_CameraPosition);

  }

  void encoder_callback(const aetos_msgs::msg::EncoderValues & msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: encoder angle1: '%f', angle2: '%f', angle3: '%f', angle4: '%f'", msg.angle1, msg.angle2, msg.angle3, msg.angle4);
    this->updateLength(msg);
  }

  void publish_motor_velocity(const sMotorVelocity & _MotorVelocity)
  {

    auto message = aetos_msgs::msg::MotorVelocity();
    message.w1=_MotorVelocity.w1;
    message.w2=_MotorVelocity.w2;
    message.w3=_MotorVelocity.w3;
    message.w4=_MotorVelocity.w4;
    pub_motor_velocity->publish(message);
  }

  void publish_position(const sPosition & _CameraPosition)
  {
    auto message = aetos_msgs::msg::EffectorPosition();
    message.position_x =_CameraPosition.x;
    message.position_y=_CameraPosition.y;
    message.position_z=_CameraPosition.z;
    pub_position->publish(message);
  }

  rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr sub_velocity;
  rclcpp::Subscription<aetos_msgs::msg::EncoderValues>::SharedPtr sub_encoder;
  rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr pub_motor_velocity;
  rclcpp::Publisher<aetos_msgs::msg::EffectorPosition>::SharedPtr pub_position;
};

void VelocityConversion::updateVelocity(const aetos_msgs::msg::Velocity & msg){
  _Velocity.vx = msg.vx;
  _Velocity.vy = msg.vy;
  _Velocity.vz = msg.vz;

  this->uavInBoundSecurityCheck();
}

void VelocityConversion::uavInBoundSecurityCheck(){
  if( (_CameraPosition.x <= 0 && _Velocity.vx < 0) || (_CameraPosition.x >= _ArenaLength && _Velocity.vx > 0)){
    _Velocity.vx = 0;
  }
  if( (_CameraPosition.y <= 0 && _Velocity.vy < 0) || (_CameraPosition.y >= _ArenaWidth && _Velocity.vy > 0)){
    _Velocity.vy = 0;
  }
  if( (_CameraPosition.z <= 0 && _Velocity.vz < 0) || (_CameraPosition.z >= _ArenaHeight && _Velocity.vz > 0)){
    _Velocity.vz = 0;
  }
};

void VelocityConversion::updateLength(const aetos_msgs::msg::EncoderValues & msg){
  _CableLength.l1 = msg.angle1*_radius + _InitialCableLength.l1;
  _CableLength.l2 = msg.angle2*_radius + _InitialCableLength.l2;
  _CableLength.l3 = msg.angle3*_radius + _InitialCableLength.l3;
  _CableLength.l4 = msg.angle4*_radius + _InitialCableLength.l4; 
}

void VelocityConversion::inverseKinematics(){
  Eigen::MatrixXf J(4,3);
  Eigen::VectorXf V(3);
  Eigen::VectorXf Lv(4);

  J(0,0) = (_CameraPosition.x - _Pole1.x)/_CableLength.l1;
  J(0,1) = (_CameraPosition.y - _Pole1.y)/_CableLength.l1;
  J(0,2) = (_CameraPosition.z - _Pole1.z)/_CableLength.l1;

  J(1,0) = (_CameraPosition.x - _Pole2.x)/_CableLength.l2;
  J(1,1) = (_CameraPosition.y - _Pole2.y)/_CableLength.l2;
  J(1,2) = (_CameraPosition.z - _Pole2.z)/_CableLength.l2;

  J(2,0) = (_CameraPosition.x - _Pole3.x)/_CableLength.l3;
  J(2,1) = (_CameraPosition.y - _Pole3.y)/_CableLength.l3;
  J(2,2) = (_CameraPosition.z - _Pole3.z)/_CableLength.l3;

  J(3,0) = (_CameraPosition.x - _Pole4.x)/_CableLength.l4;
  J(3,1) = (_CameraPosition.y - _Pole4.y)/_CableLength.l4;
  J(3,2) = (_CameraPosition.z - _Pole4.z)/_CableLength.l4;

  V(0) = _Velocity.vx;
  V(1) = _Velocity.vy;
  V(2) = _Velocity.vz;

  Lv = J*V;
  std::cout << "Vitesse en rad/s: " << Lv << std::endl;


  _MotorVelocity.w1 = Lv(0)/(_radius);
  _MotorVelocity.w2 = Lv(1)/(_radius);
  _MotorVelocity.w3 = Lv(2)/(_radius);
  _MotorVelocity.w4 = Lv(3)/(_radius);


}
void VelocityConversion::forwardKinematics(){
  _CameraPosition.x = (_CableLength.l1 * _CableLength.l1 + _ArenaLength * _ArenaLength - _CableLength.l4 * _CableLength.l4) / (2 * _ArenaLength);
  _CameraPosition.y = (_CableLength.l1 * _CableLength.l1 + _ArenaWidth * _ArenaWidth - _CableLength.l2 * _CableLength.l2) / (2 * _ArenaWidth);
  _CameraPosition.z = sqrt(abs(_CableLength.l1 * _CableLength.l1 - _CameraPosition.x * _CameraPosition.x - _CameraPosition.y * _CameraPosition.y));
  std::cout << "Position de la camera: " << _CameraPosition.x << " " << _CameraPosition.y << " " << _CameraPosition.z << std::endl;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityConversion>());
  rclcpp::shutdown();
  return 0;
}
