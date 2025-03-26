#include <functional>
#include <memory>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "aetos_msgs/msg/velocity.hpp"
#include "aetos_msgs/msg/encoder_values.hpp"
#include "aetos_msgs/msg/motor_velocity.hpp"
#include "aetos_msgs/msg/effector_position.hpp"

using std::placeholders::_1;

struct sVelocity
{
  float vx;
  float vy;
  float vz;
};

struct sPosition
{
  float x;
  float y;
  float z;
};

struct sMotorVelocity
{
  float w1;
  float w2;
  float w3;
  float w4;
};

struct sCableLength
{
  float l1;
  float l2;
  float l3;
  float l4;
};

constexpr float LIMIT_BUFFER = 0.2f;

namespace Limits
{
  constexpr float X_MAX = 1.0f - LIMIT_BUFFER;
  constexpr float X_MIN = 0.0f + LIMIT_BUFFER;
  constexpr float Y_MAX = 1.0f - LIMIT_BUFFER;
  constexpr float Y_MIN = 0.0f + LIMIT_BUFFER;
  constexpr float Z_MAX = 1.0f - LIMIT_BUFFER;
  constexpr float Z_MIN = 0.0f + LIMIT_BUFFER;
}

// SETUP
constexpr float PI = 3.14159265359;

constexpr float MAX_WHEEL_VELOCITY = 6.0f;

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
    _subJoyVelocity = this->create_subscription<aetos_msgs::msg::Velocity>(
        "aetos/control/velocity", 10, std::bind(&VelocityConversion::joyVelocityCallback, this, _1));

    _subEncoder = this->create_subscription<aetos_msgs::msg::EncoderValues>(
        "aetos/control/encoder", 10, std::bind(&VelocityConversion::encoder_callback, this, _1));

    _joyVelPub = this->create_publisher<aetos_msgs::msg::MotorVelocity>("aetos/cmd/velocity", 10);

    _angularVelPub = this->create_publisher<aetos_msgs::msg::MotorVelocity>("aetos/control/angular_velocity", 10);

    _positionPub = this->create_publisher<aetos_msgs::msg::EffectorPosition>("aetos/control/position", 10);
  }

private:
  sVelocity _velocity;
  sPosition _cameraPosition;
  sMotorVelocity _motorVelocity;
  sCableLength _cableLength;

  void updateVelocity(const aetos_msgs::msg::Velocity &msg);
  void updateLength(const aetos_msgs::msg::EncoderValues &msg);
  void forwardKinematics();
  void inverseKinematics();
  void uavInBoundSecurityCheck();
  void scaleVelocities();

  void autoVelocityCallback(const aetos_msgs::msg::Velocity &msg)
  {
    auto message = aetos_msgs::msg::MotorVelocity();

    float vx = msg.velocity_x;
    float vy = msg.velocity_y;
    float vz = msg.velocity_z;

    _velocity.vx = vx;
    _velocity.vy = vy;
    _velocity.vz = vz;

    this->forwardKinematics();
    this->uavInBoundSecurityCheck();
    this->inverseKinematics();

    message.omega1 = _motorVelocity.w1;
    message.omega2 = _motorVelocity.w2;
    message.omega3 = _motorVelocity.w3;
    message.omega4 = _motorVelocity.w4;

    _autoVelPub->publish(message);
    this->publish_position(_cameraPosition);
  }

  void joyVelocityCallback(const aetos_msgs::msg::Velocity &msg)
  {
    auto message = aetos_msgs::msg::MotorVelocity();

    float vx = msg.velocity_x;
    float vy = msg.velocity_y;
    float vz = msg.velocity_z;

    _velocity.vx = vx;
    _velocity.vy = vy;
    _velocity.vz = vz;

    this->forwardKinematics();
    this->uavInBoundSecurityCheck();
    this->inverseKinematics();

    message.omega1 = _motorVelocity.w1;
    message.omega2 = _motorVelocity.w2;
    message.omega3 = _motorVelocity.w3;
    message.omega4 = _motorVelocity.w4;
    _joyVelPub->publish(message);
    _angularVelPub->publish(message);

    this->publish_position(_cameraPosition);
  }

  void encoder_callback(const aetos_msgs::msg::EncoderValues &msg)
  {
    this->updateLength(msg);
  }

  void publish_motor_velocity(const sMotorVelocity &_motorVelocity)
  {
    auto message = aetos_msgs::msg::MotorVelocity();
    message.omega1 = _motorVelocity.w1;
    message.omega2 = _motorVelocity.w2;
    message.omega3 = _motorVelocity.w3;
    message.omega4 = _motorVelocity.w4;
    _joyVelPub->publish(message);
  }

  void publish_position(const sPosition &_cameraPosition)
  {
    auto message = aetos_msgs::msg::EffectorPosition();
    message.position_x = _cameraPosition.x;
    message.position_y = _cameraPosition.y;
    message.position_z = _cameraPosition.z;
    _positionPub->publish(message);
  }

  rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr _subJoyVelocity;
  rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr _subAutoVelocity;
  rclcpp::Subscription<aetos_msgs::msg::EncoderValues>::SharedPtr _subEncoder;

  rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr _joyVelPub;
  rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr _autoVelPub;
  rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr _angularVelPub;
  rclcpp::Publisher<aetos_msgs::msg::EffectorPosition>::SharedPtr _positionPub;
};

void VelocityConversion::updateVelocity(const aetos_msgs::msg::Velocity &msg)
{
  _velocity.vx = msg.velocity_x;
  _velocity.vy = msg.velocity_y;
  _velocity.vz = msg.velocity_z;

  this->uavInBoundSecurityCheck();
}

void VelocityConversion::uavInBoundSecurityCheck()
{

  if ((_cameraPosition.x <= Limits::X_MIN && _velocity.vx < 0) || (_cameraPosition.x >= Limits::X_MAX && _velocity.vx > 0))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hit Limits");
    _velocity.vx = 0;
  }
  else if ((_cameraPosition.y <= Limits::Y_MIN && _velocity.vy < 0) || (_cameraPosition.y >= Limits::Y_MAX && _velocity.vy > 0))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hit Limits");
    _velocity.vy = 0;
  }
  else if ((_cameraPosition.z <= Limits::Z_MIN && _velocity.vz < 0) || (_cameraPosition.z >= Limits::Z_MAX && _velocity.vz > 0))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hit Limits");
    _velocity.vz = 0;
  }
};

void VelocityConversion::updateLength(const aetos_msgs::msg::EncoderValues &msg)
{
  _cableLength.l1 = msg.angle1 * _radius + _initialCableLength.l1;
  _cableLength.l2 = msg.angle2 * _radius + _initialCableLength.l2;
  _cableLength.l3 = msg.angle3 * _radius + _initialCableLength.l3;
  _cableLength.l4 = msg.angle4 * _radius + _initialCableLength.l4;
}

void VelocityConversion::inverseKinematics()
{
  Eigen::MatrixXf J(4, 3);
  Eigen::VectorXf V(3);
  Eigen::VectorXf Lv(4);

  J(0, 0) = (_cameraPosition.x - _pole1.x) / _cableLength.l1;
  J(0, 1) = (_cameraPosition.y - _pole1.y) / _cableLength.l1;
  J(0, 2) = (_cameraPosition.z - _pole1.z) / _cableLength.l1;

  J(1, 0) = (_cameraPosition.x - _pole2.x) / _cableLength.l2;
  J(1, 1) = (_cameraPosition.y - _pole2.y) / _cableLength.l2;
  J(1, 2) = (_cameraPosition.z - _pole2.z) / _cableLength.l2;

  J(2, 0) = (_cameraPosition.x - _pole3.x) / _cableLength.l3;
  J(2, 1) = (_cameraPosition.y - _pole3.y) / _cableLength.l3;
  J(2, 2) = (_cameraPosition.z - _pole3.z) / _cableLength.l3;

  J(3, 0) = (_cameraPosition.x - _pole4.x) / _cableLength.l4;
  J(3, 1) = (_cameraPosition.y - _pole4.y) / _cableLength.l4;
  J(3, 2) = (_cameraPosition.z - _pole4.z) / _cableLength.l4;

  V(0) = _velocity.vx;
  V(1) = _velocity.vy;
  V(2) = _velocity.vz;

  Lv = J * V;

  _motorVelocity.w1 = Lv(0) / (_radius);
  _motorVelocity.w2 = Lv(1) / (_radius);
  _motorVelocity.w3 = Lv(2) / (_radius);
  _motorVelocity.w4 = Lv(3) / (_radius);

  float velocityRatio = std::max({abs(Lv(0) / _radius / MAX_WHEEL_VELOCITY),
                                  abs(Lv(1) / _radius / MAX_WHEEL_VELOCITY),
                                  abs(Lv(2) / _radius / MAX_WHEEL_VELOCITY),
                                  abs(Lv(3) / _radius / MAX_WHEEL_VELOCITY)});

  // Scale velocities if needed
  if (velocityRatio > 1.0f)
  {
    Lv /= velocityRatio;
  }

  // Assign scaled velocities to motor velocities
  _motorVelocity.w1 = Lv(0) / _radius;
  _motorVelocity.w2 = Lv(1) / _radius;
  _motorVelocity.w3 = Lv(2) / _radius;
  _motorVelocity.w4 = Lv(3) / _radius;
}
void VelocityConversion::forwardKinematics()
{
  _cameraPosition.x = (_cableLength.l1 * _cableLength.l1 + _arenaLength * _arenaLength - _cableLength.l4 * _cableLength.l4) / (2 * _arenaLength);
  _cameraPosition.y = (_cableLength.l1 * _cableLength.l1 + _arenaWidth * _arenaWidth - _cableLength.l2 * _cableLength.l2) / (2 * _arenaWidth);
  _cameraPosition.z = sqrt(abs(_cableLength.l1 * _cableLength.l1 - _cameraPosition.x * _cameraPosition.x - _cameraPosition.y * _cameraPosition.y));
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityConversion>());
  rclcpp::shutdown();
  return 0;
}
