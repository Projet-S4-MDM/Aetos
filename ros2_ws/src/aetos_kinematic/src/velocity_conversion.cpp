#include <functional>
#include <memory>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "aetos_msgs/msg/velocity.hpp"
#include "aetos_msgs/msg/encoder_values.hpp" 
#include "aetos_msgs/msg/motor_velocity.hpp"  
#include "aetos_msgs/msg/effector_position.hpp"

using std::placeholders::_1;

struct sVelocity{
  float vx;
  float vy;
  float vz;
};

struct sPosition{
  float x;
  float y;
  float z;
};

struct sMotorVelocity{
  float w1;
  float w2;
  float w3;
  float w4;
};

struct sCableLength{
  float l1;
  float l2;
  float l3;
  float l4;
};

  //SETUP
  constexpr float PI = 3.14159265359;

  constexpr float _radius = 0.05;

  constexpr float _arenaLength = 0.816;
  constexpr float _arenaWidth = 0.816;
  constexpr float _arenaHeight = 0.7;

  constexpr sCableLength _initialCableLength = {0.05f, 0.78144487f, 1.10399827f, 0.78144487f};

  constexpr sPosition _pole1 = {0.0f, 0.0f, 0.0f};
  constexpr sPosition _pole2 = {0.0f, 0.816f, 0.0f};
  constexpr sPosition _pole3 = {0.816f, 0.816f, 0.0f};
  constexpr sPosition _pole4 = {0.816f, 0.0f, 0.0f};




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

  sVelocity _velocity;
  sPosition _cameraPosition;
  sMotorVelocity _motorVelocity;
  sCableLength _cableLength;

  
  void updateVelocity(const aetos_msgs::msg::Velocity & msg);

  void updateLength(const aetos_msgs::msg::EncoderValues & msg);

  void forwardKinematics();

  void inverseKinematics();

  void uavInBoundSecurityCheck();

  void velocity_callback(const aetos_msgs::msg::Velocity & msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: velocity vx: '%d', vy: '%d', vz: '%d'", msg.VX, msg.VY, msg.VZ);
    this->updateVelocity(msg);
    this->forwardKinematics();
    this->inverseKinematics();
    this->publish_motor_velocity(_motorVelocity);
    this->publish_position(_cameraPosition);

  }

  void encoder_callback(const aetos_msgs::msg::EncoderValues & msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: encoder angle1: '%f', angle2: '%f', angle3: '%f', angle4: '%f'", msg.angle1, msg.angle2, msg.angle3, msg.angle4);
    this->updateLength(msg);
  }

  void publish_motor_velocity(const sMotorVelocity & _motorVelocity)
  {

    auto message = aetos_msgs::msg::MotorVelocity();
    message.w1 = _motorVelocity.w1;
    message.w2 = _motorVelocity.w2;
    message.w3 = _motorVelocity.w3;
    message.w4 = _motorVelocity.w4;
    pub_motor_velocity->publish(message);
  }

  void publish_position(const sPosition & _cameraPosition)
  {
    auto message = aetos_msgs::msg::EffectorPosition();
    message.position_x = _cameraPosition.x;
    message.position_y = _cameraPosition.y;
    message.position_z = _cameraPosition.z;
    pub_position->publish(message);
  }

  rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr sub_velocity;
  rclcpp::Subscription<aetos_msgs::msg::EncoderValues>::SharedPtr sub_encoder;
  rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr pub_motor_velocity;
  rclcpp::Publisher<aetos_msgs::msg::EffectorPosition>::SharedPtr pub_position;
};

void VelocityConversion::updateVelocity(const aetos_msgs::msg::Velocity & msg){
  _velocity.vx = msg.VX;
  _velocity.vy = msg.VY;
  _velocity.vz = msg.VZ;

  this->uavInBoundSecurityCheck();
}

void VelocityConversion::uavInBoundSecurityCheck(){
  if( (_cameraPosition.x <= 0 && _velocity.vx < 0) || (_cameraPosition.x >= _arenaLength && _velocity.vx > 0)){
    _velocity.vx = 0;
  }
  if( (_cameraPosition.y <= 0 && _velocity.vy < 0) || (_cameraPosition.y >= _arenaWidth && _velocity.vy > 0)){
    _velocity.vy = 0;
  }
  if( (_cameraPosition.z <= 0 && _velocity.vz < 0) || (_cameraPosition.z >= _arenaHeight && _velocity.vz > 0)){
    _velocity.vz = 0;
  }
};

void VelocityConversion::updateLength(const aetos_msgs::msg::EncoderValues & msg){
  _cableLength.l1 = msg.angle1*_radius + _initialCableLength.l1;
  _cableLength.l2 = msg.angle2*_radius + _initialCableLength.l2;
  _cableLength.l3 = msg.angle3*_radius + _initialCableLength.l3;
  _cableLength.l4 = msg.angle4*_radius + _initialCableLength.l4; 
}

void VelocityConversion::inverseKinematics(){
  Eigen::MatrixXf J(4,3);
  Eigen::VectorXf V(3);
  Eigen::VectorXf Lv(4);

  J(0,0) = (_cameraPosition.x - _pole1.x)/_cableLength.l1;
  J(0,1) = (_cameraPosition.y - _pole1.y)/_cableLength.l1;
  J(0,2) = (_cameraPosition.z - _pole1.z)/_cableLength.l1;

  J(1,0) = (_cameraPosition.x - _pole2.x)/_cableLength.l2;
  J(1,1) = (_cameraPosition.y - _pole2.y)/_cableLength.l2;
  J(1,2) = (_cameraPosition.z - _pole2.z)/_cableLength.l2;

  J(2,0) = (_cameraPosition.x - _pole3.x)/_cableLength.l3;
  J(2,1) = (_cameraPosition.y - _pole3.y)/_cableLength.l3;
  J(2,2) = (_cameraPosition.z - _pole3.z)/_cableLength.l3;

  J(3,0) = (_cameraPosition.x - _pole4.x)/_cableLength.l4;
  J(3,1) = (_cameraPosition.y - _pole4.y)/_cableLength.l4;
  J(3,2) = (_cameraPosition.z - _pole4.z)/_cableLength.l4;

  V(0) = _velocity.vx;
  V(1) = _velocity.vy;
  V(2) = _velocity.vz;

  Lv = J*V;
  std::cout << "Vitesse en rad/s: " << Lv << std::endl;


  _motorVelocity.w1 = Lv(0)/(_radius);
  _motorVelocity.w2 = Lv(1)/(_radius);
  _motorVelocity.w3 = Lv(2)/(_radius);
  _motorVelocity.w4 = Lv(3)/(_radius);


}
void VelocityConversion::forwardKinematics(){
  _cameraPosition.x = (_cableLength.l1 * _cableLength.l1 + _arenaLength * _arenaLength - _cableLength.l4 * _cableLength.l4) / (2 * _arenaLength);
  _cameraPosition.y = (_cableLength.l1 * _cableLength.l1 + _arenaWidth * _arenaWidth - _cableLength.l2 * _cableLength.l2) / (2 * _arenaWidth);
  _cameraPosition.z = sqrt(abs(_cableLength.l1 * _cableLength.l1 - _cameraPosition.x * _cameraPosition.x - _cameraPosition.y * _cameraPosition.y));
  std::cout << "Position de la camera: " << _cameraPosition.x << " " << _cameraPosition.y << " " << _cameraPosition.z << std::endl;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityConversion>());
  rclcpp::shutdown();
  return 0;
}
