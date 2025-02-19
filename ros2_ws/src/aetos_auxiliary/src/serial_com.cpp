#include <rclcpp/rclcpp.hpp>
#include <aetos_msgs/msg/velocity.hpp>

#include <boost/asio.hpp>
#include <iostream>
#include <string>

class SerialCom : public rclcpp::Node
{
public:
    SerialCom();
    ~SerialCom() {};

private:
    void topicCallback(const aetos_msgs::msg::Velocity::SharedPtr msg)
    {
        try
        {
            boost::asio::write(serial_, boost::asio::buffer(&msg->data[msg->VX], sizeof(float)));
            boost::asio::write(serial_, boost::asio::buffer(&msg->data[msg->VY], sizeof(float)));
            boost::asio::write(serial_, boost::asio::buffer(&msg->data[msg->VZ], sizeof(float)));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
        }
    }

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_;

    rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr _motorVelSub;
};

SerialCom::SerialCom() : Node("serial_comm"), serial_(io_service_)
{
    try
    {
        serial_.open("/dev/ttyUSB0");
        serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        rclcpp::shutdown();
    }

    _motorVelSub = this->create_subscription<aetos_msgs::msg::Velocity>(
        "aetos/control/motor_speed", 10, std::bind(&SerialCom::topicCallback, this, std::placeholders::_1));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCom>());
    rclcpp::shutdown();

    return 0;
}
