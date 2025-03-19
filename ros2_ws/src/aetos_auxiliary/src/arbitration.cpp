#include <rclcpp/rclcpp.hpp>

#include <aetos_msgs/msg/motor_velocity.hpp>
#include <aetos_msgs/msg/velocity_arbitration.hpp>
#include <aetos_msgs/srv/velocity_arbitration.hpp>

constexpr uint8_t N_JOINTS = 4;

class Arbitration : public rclcpp::Node
{
public:
    Arbitration();
    ~Arbitration() {};

private:
    void teleopCmdCallback(const aetos_msgs::msg::MotorVelocity teleopCmdMsg_);
    void autoCmdCallback(const aetos_msgs::msg::MotorVelocity autoCmdMsg_);
    void arbitrationCallback(const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Request> request_,
                             const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Response> response_);
    void sendCmd(void);

    rclcpp::Service<aetos_msgs::srv::VelocityArbitration>::SharedPtr _velocitySrv;
    rclcpp::Subscription<aetos_msgs::msg::MotorVelocity>::SharedPtr _teleopCmdVel;
    rclcpp::Subscription<aetos_msgs::msg::MotorVelocity>::SharedPtr _autoCmdVel;
    rclcpp::Publisher<aetos_msgs::msg::MotorVelocity>::SharedPtr _motorVelPub;
    rclcpp::TimerBase::SharedPtr _cmdSendTimer;

    aetos_msgs::msg::MotorVelocity _zeroCmd;
    aetos_msgs::msg::MotorVelocity _cmdTeleop;
    aetos_msgs::msg::MotorVelocity _cmdAuto;

    aetos_msgs::msg::VelocityArbitration _arbitration;
};

Arbitration::Arbitration() : Node("arbitration_node")
{
    for (size_t i = 0; i < N_JOINTS; i++)
    {
        _zeroCmd.omega1 = 0.0f;
        _zeroCmd.omega2 = 0.0f;
        _zeroCmd.omega3 = 0.0f;
        _zeroCmd.omega4 = 0.0f;

        _cmdAuto.omega1 = 0.0f;
        _cmdAuto.omega2 = 0.0f;
        _cmdAuto.omega3 = 0.0f;
        _cmdAuto.omega4 = 0.0f;

        _cmdAuto.omega1 = 0.0f;
        _cmdAuto.omega2 = 0.0f;
        _cmdAuto.omega3 = 0.0f;
        _cmdAuto.omega4 = 0.0f;
    }

    _teleopCmdVel = this->create_subscription<aetos_msgs::msg::MotorVelocity>(
        "aetos/control/teleop", 10,
        std::bind(&Arbitration::teleopCmdCallback, this, std::placeholders::_1));
    _autoCmdVel = this->create_subscription<aetos_msgs::msg::MotorVelocity>(
        "aetos/control/auto", 10,
        std::bind(&Arbitration::autoCmdCallback, this, std::placeholders::_1));

    _motorVelPub = this->create_publisher<aetos_msgs::msg::MotorVelocity>(
        "aetos/control/velocity", 1);
    _velocitySrv = this->create_service<aetos_msgs::srv::VelocityArbitration>(
        "aetos/communication/set_arbitration",
        std::bind(&Arbitration::arbitrationCallback, this, std::placeholders::_1, std::placeholders::_2));
    _cmdSendTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Arbitration::sendCmd, this));
}

void Arbitration::arbitrationCallback(const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Request> request_,
                                      const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Response> response_)
{
    if (request_->target_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::NONE || request_->target_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::TELEOP || request_->target_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::AUTONOMUS)
    {
        _arbitration = request_->target_arbitration;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Desired arbitration not implemented");
    }

    response_->current_arbitration = _arbitration;
}

void Arbitration::teleopCmdCallback(const aetos_msgs::msg::MotorVelocity teleopCmdMsg_)
{
    _cmdTeleop = teleopCmdMsg_;
}

void Arbitration::autoCmdCallback(const aetos_msgs::msg::MotorVelocity autoCmdMsg_)
{
    _cmdAuto = autoCmdMsg_;
}

void Arbitration::sendCmd(void)
{
    if (_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::TELEOP)
    {
        _motorVelPub->publish(_cmdTeleop);
    }
    else if (_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::AUTONOMUS)
    {
        _motorVelPub->publish(_cmdAuto);
    }
    else
    {
        _motorVelPub->publish(_zeroCmd);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();

    return 0;
}