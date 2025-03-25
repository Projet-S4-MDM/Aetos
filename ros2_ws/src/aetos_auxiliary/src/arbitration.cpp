#include <rclcpp/rclcpp.hpp>

#include <aetos_msgs/msg/velocity.hpp>
#include <aetos_msgs/msg/motor_velocity.hpp>
#include <aetos_msgs/msg/encoder_values.hpp>
#include <aetos_msgs/msg/velocity_arbitration.hpp>
#include <aetos_msgs/msg/encoder_arbitration.hpp>
#include <aetos_msgs/srv/velocity_arbitration.hpp>
#include <aetos_msgs/srv/encoder_arbitration.hpp>

constexpr uint8_t N_JOINTS = 4;

class Arbitration : public rclcpp::Node
{
public:
    Arbitration();
    ~Arbitration() = default;

private:
    void teleopCmdCallback(const aetos_msgs::msg::Velocity teleopCmdMsg_);
    void autoCmdCallback(const aetos_msgs::msg::Velocity autoCmdMsg_);
    void simEncoderCallback(const aetos_msgs::msg::EncoderValues simEncoderMsg_);
    void motorEncoderCallback(const aetos_msgs::msg::EncoderValues motorEncoderMsg_);

    void velocityArbitrationCallback(const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Request> request_,
                                     const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Response> response_);
    void encoderArbitrationCallback(const std::shared_ptr<aetos_msgs::srv::EncoderArbitration::Request> request_,
                                    const std::shared_ptr<aetos_msgs::srv::EncoderArbitration::Response> response_);

    void sendCmd(void);

    rclcpp::Service<aetos_msgs::srv::VelocityArbitration>::SharedPtr _velocitySrv;
    rclcpp::Service<aetos_msgs::srv::EncoderArbitration>::SharedPtr _encoderSrv;

    rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr _teleopCmdSub;
    rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr _autoCmdSub;
    rclcpp::Subscription<aetos_msgs::msg::EncoderValues>::SharedPtr _simEncoderSub;
    rclcpp::Subscription<aetos_msgs::msg::EncoderValues>::SharedPtr _motorEncoderSub;

    rclcpp::Publisher<aetos_msgs::msg::Velocity>::SharedPtr _motorVelPub;
    rclcpp::Publisher<aetos_msgs::msg::EncoderValues>::SharedPtr _encoderPub;

    rclcpp::TimerBase::SharedPtr _cmdSendTimer;

    aetos_msgs::msg::Velocity _zeroCmd;
    aetos_msgs::msg::Velocity _cmdTeleop;
    aetos_msgs::msg::Velocity _cmdAuto;

    aetos_msgs::msg::EncoderValues _zeroEncoder;
    aetos_msgs::msg::EncoderValues _simEncoder;
    aetos_msgs::msg::EncoderValues _motorEncoder;

    aetos_msgs::msg::VelocityArbitration _velocityArbitration;
    aetos_msgs::msg::EncoderArbitration _encoderArbitration;
};

Arbitration::Arbitration() : Node("arbitration_node")
{
    // for (size_t i = 0; i < N_JOINTS; i++)
    // {
    //     _zeroCmd.omega1 = 0.0f;
    //     _zeroCmd.omega2 = 0.0f;
    //     _zeroCmd.omega3 = 0.0f;
    //     _zeroCmd.omega4 = 0.0f;

    //     _cmdAuto.omega1 = 0.0f;
    //     _cmdAuto.omega2 = 0.0f;
    //     _cmdAuto.omega3 = 0.0f;
    //     _cmdAuto.omega4 = 0.0f;

    //     _cmdTeleop.omega1 = 0.0f;
    //     _cmdTeleop.omega2 = 0.0f;
    //     _cmdTeleop.omega3 = 0.0f;
    //     _cmdTeleop.omega4 = 0.0f;
    // }

    _teleopCmdSub = this->create_subscription<aetos_msgs::msg::Velocity>(
        "aetos/velocity/teleop", 10,
        std::bind(&Arbitration::teleopCmdCallback, this, std::placeholders::_1));
    _autoCmdSub = this->create_subscription<aetos_msgs::msg::Velocity>(
        "aetos/velocity/auto", 10,
        std::bind(&Arbitration::autoCmdCallback, this, std::placeholders::_1));
    _motorEncoderSub = this->create_subscription<aetos_msgs::msg::EncoderValues>(
        "aetos/encoder/motor", 10,
        std::bind(&Arbitration::motorEncoderCallback, this, std::placeholders::_1));
    _simEncoderSub = this->create_subscription<aetos_msgs::msg::EncoderValues>(
        "aetos/encoder/sim", 10,
        std::bind(&Arbitration::simEncoderCallback, this, std::placeholders::_1));

    _motorVelPub = this->create_publisher<aetos_msgs::msg::Velocity>(
        "aetos/control/velocity", 1);
    _encoderPub = this->create_publisher<aetos_msgs::msg::EncoderValues>(
        "aetos/control/encoder", 1);

    _velocitySrv = this->create_service<aetos_msgs::srv::VelocityArbitration>(
        "aetos/communication/set_velocityArbitration",
        std::bind(&Arbitration::velocityArbitrationCallback, this, std::placeholders::_1, std::placeholders::_2));
    _encoderSrv = this->create_service<aetos_msgs::srv::EncoderArbitration>(
        "aetos/communication/set_encoderArbitration",
        std::bind(&Arbitration::encoderArbitrationCallback, this, std::placeholders::_1, std::placeholders::_2));

    _cmdSendTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Arbitration::sendCmd, this));
}

void Arbitration::velocityArbitrationCallback(const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Request> request_,
                                              const std::shared_ptr<aetos_msgs::srv::VelocityArbitration::Response> response_)
{
    if (request_->target_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::NONE || request_->target_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::TELEOP || request_->target_arbitration.arbitration == aetos_msgs::msg::VelocityArbitration::AUTONOMUS)
    {
        _velocityArbitration = request_->target_arbitration;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Desired arbitration not implemented");
    }

    response_->current_arbitration = _velocityArbitration;
}

void Arbitration::encoderArbitrationCallback(const std::shared_ptr<aetos_msgs::srv::EncoderArbitration::Request> request_,
                                             const std::shared_ptr<aetos_msgs::srv::EncoderArbitration::Response> response_)
{
    if (request_->target_arbitration.arbitration == aetos_msgs::msg::EncoderArbitration::NONE || request_->target_arbitration.arbitration == aetos_msgs::msg::EncoderArbitration::MOTOR_ENCODER || request_->target_arbitration.arbitration == aetos_msgs::msg::EncoderArbitration::SIM_ENCODER)
    {
        _encoderArbitration = request_->target_arbitration;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Desired arbitration not implemented");
    }

    response_->current_arbitration = _encoderArbitration;
}

void Arbitration::teleopCmdCallback(const aetos_msgs::msg::Velocity teleopCmdMsg_)
{
    _cmdTeleop = teleopCmdMsg_;
}

void Arbitration::autoCmdCallback(const aetos_msgs::msg::Velocity autoCmdMsg_)
{
    _cmdAuto = autoCmdMsg_;
}

void Arbitration::simEncoderCallback(const aetos_msgs::msg::EncoderValues simEncoderMsg_)
{
    _simEncoder = simEncoderMsg_;
}

void Arbitration::motorEncoderCallback(const aetos_msgs::msg::EncoderValues motorEncoderMsg_)
{
    _motorEncoder = motorEncoderMsg_;
}

void Arbitration::sendCmd(void)
{
    switch (_velocityArbitration.arbitration)
    {
    case aetos_msgs::msg::VelocityArbitration::TELEOP:
        _motorVelPub->publish(_cmdTeleop);
        break;
    case aetos_msgs::msg::VelocityArbitration::AUTONOMUS:
        _motorVelPub->publish(_cmdAuto);
        break;
    case aetos_msgs::msg::VelocityArbitration::NONE:
        _motorVelPub->publish(_zeroCmd);
        break;
    default:
        break;
    }

    switch (_encoderArbitration.arbitration)
    {
    case aetos_msgs::msg::EncoderArbitration::MOTOR_ENCODER:
        _encoderPub->publish(_motorEncoder);
        break;
    case aetos_msgs::msg::EncoderArbitration::SIM_ENCODER:
        _encoderPub->publish(_simEncoder);
        break;
    case aetos_msgs::msg::EncoderArbitration::NONE:
        _encoderPub->publish(_zeroEncoder);
        break;
    default:
        break;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();

    return 0;
}