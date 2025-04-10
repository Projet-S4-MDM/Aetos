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

constexpr float LIMIT_BUFFER = 0.1f;
constexpr float LIMIT_Z = 0.1f;

namespace Limits
{
  constexpr float X_MAX = 1.0f - LIMIT_BUFFER;
  constexpr float X_MIN = 0.0f + LIMIT_BUFFER;
  constexpr float Y_MAX = 1.0f - LIMIT_BUFFER;
  constexpr float Y_MIN = 0.0f + LIMIT_BUFFER;
  constexpr float Z_MAX = LIMIT_Z;
  constexpr float Z_MIN = 0.90f;
}

// SETUP
constexpr float PI = 3.14159265359;

constexpr float MAX_WHEEL_VELOCITY = 5.0f;

constexpr float MAX_ACCEL = 3.0f;

constexpr float _radius = 0.05;

constexpr float _arenaLength = 0.95f;
constexpr float _arenaWidth = 0.95f;
constexpr float _arenaHeight = 0.935f;

constexpr sCableLength _initialCableLength = {1.037f, 1.037f, 1.037f, 1.037f};

constexpr sPosition _pole1 = {0.0f, 0.0f, 0.0f};
constexpr sPosition _pole2 = {0.0f, 0.95f, 0.0f};
constexpr sPosition _pole3 = {0.95f, 0.95f, 0.0f};
constexpr sPosition _pole4 = {0.95f, 0.0f, 0.0f};

sVelocity _currentVelocity = {0.0f, 0.0f, 0.0f};

rclcpp::Time _lastVelocityUpdateTime{0, 0, RCL_ROS_TIME};

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
  sPosition _positionError = {0.0f, 0.0f, 0.0f};
  sMotorVelocity _motorVelocity;
  sCableLength _cableLength;
  sPosition _desiredPosition = {0.475f, 0.475f, 0.79f};

  void updateVelocity(const aetos_msgs::msg::Velocity &msg);
  void updateLength(const aetos_msgs::msg::EncoderValues &msg);
  void forwardKinematics();
  void inverseKinematics();
  void uavInBoundSecurityCheck();
  void positionError();
  void updateDesiredPosition();

  void joyVelocityCallback(const aetos_msgs::msg::Velocity &msg)
  {
    auto message = aetos_msgs::msg::MotorVelocity();

    rclcpp::Time currentTime = this->now();
    double dt = 0.01; 

    if (_lastVelocityUpdateTime.nanoseconds() > 0)
    {
      dt = (currentTime - _lastVelocityUpdateTime).seconds();

      if (dt > 0.1)
      {
        dt = 0.1f;
            }
      if (dt <= 0.0)
      {
        dt = 0.01f;
      }
    }

    _lastVelocityUpdateTime = currentTime;

    sVelocity targetVelocity = {msg.velocity_x, msg.velocity_y, msg.velocity_z};

    float max_velocity_change = MAX_ACCEL * dt;

    float vx_diff = targetVelocity.vx - _currentVelocity.vx;
    if (std::abs(vx_diff) > max_velocity_change)
    {
      _currentVelocity.vx += (vx_diff > 0) ? max_velocity_change : -max_velocity_change;
    }
    else
    {
      _currentVelocity.vx = targetVelocity.vx;
    }

    float vy_diff = targetVelocity.vy - _currentVelocity.vy;
    if (std::abs(vy_diff) > max_velocity_change)
    {
      _currentVelocity.vy += (vy_diff > 0) ? max_velocity_change : -max_velocity_change;
    }
    else
    {
      _currentVelocity.vy = targetVelocity.vy;
    }

    float vz_diff = targetVelocity.vz - _currentVelocity.vz;
    if (std::abs(vz_diff) > max_velocity_change)
    {
      _currentVelocity.vz += (vz_diff > 0) ? max_velocity_change : -max_velocity_change;
    }
    else
    {
      _currentVelocity.vz = targetVelocity.vz;
    }

    _velocity.vx = _currentVelocity.vx;
    _velocity.vy = _currentVelocity.vy;
    _velocity.vz = _currentVelocity.vz;

    this->updateDesiredPosition();
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
    _velocity.vx = 0.0f;
  }
  else if ((_cameraPosition.y <= Limits::Y_MIN && _velocity.vy < 0) || (_cameraPosition.y >= Limits::Y_MAX && _velocity.vy > 0))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hit Limits");
    _velocity.vy = 0.0f;
  }
};

void VelocityConversion::updateLength(const aetos_msgs::msg::EncoderValues &msg)
{
  _cableLength.l1 = msg.angle1 * _radius + _initialCableLength.l1;
  _cableLength.l2 = msg.angle2 * _radius + _initialCableLength.l2;
  _cableLength.l3 = msg.angle3 * _radius + _initialCableLength.l3;
  _cableLength.l4 = msg.angle4 * _radius + _initialCableLength.l4;
}

void VelocityConversion::positionError()
{
  _positionError.x = _desiredPosition.x - _cameraPosition.x;
  _positionError.y = _desiredPosition.y - _cameraPosition.y;
  _positionError.z = _desiredPosition.z - _cameraPosition.z;

  const double threshold = 0.001;

  if (std::abs(_positionError.x) < threshold)
    _positionError.x = 0.0;
  if (std::abs(_positionError.y) < threshold)
    _positionError.y = 0.0;
  if (std::abs(_positionError.z) < threshold)
    _positionError.z = 0.0;

  const float Kp = 0.6f;

  _velocity.vx += Kp * _positionError.x;
  _velocity.vy += Kp * _positionError.y;
  _velocity.vz += Kp * _positionError.z;
}

void VelocityConversion::updateDesiredPosition()
{
  float dt = 0.01f;

  _desiredPosition.x += _velocity.vx * dt;
  _desiredPosition.y += _velocity.vy * dt;
  _desiredPosition.z += _velocity.vz * dt;
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

  float velocityRatio = std::max({abs(Lv(0) / _radius / MAX_WHEEL_VELOCITY),
                                  abs(Lv(1) / _radius / MAX_WHEEL_VELOCITY),
                                  abs(Lv(2) / _radius / MAX_WHEEL_VELOCITY),
                                  abs(Lv(3) / _radius / MAX_WHEEL_VELOCITY)});

  if (velocityRatio > 1.0f)
  {
    Lv /= velocityRatio;
  }

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
