#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"

#include <boost/asio.hpp>
#include <iostream>
#include <string>

class SerialCom : public rclcpp::Node
{
public:
    SerialCom();

private:
    void topicCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::string data = msg->data;

        try
        {
            boost::asio::write(serial_, boost::asio::buffer(data));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
        }
    }

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _motorVelSub;
};

SerialCom::SerialCom() : Node("serial_comm"), serial_(io_service_)
{
    try
    {
        serial_.open("/dev/ttyUSB0");
        serial_.set_option(boost::asio::serial_port_base::baud_rate(460800));
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

    _motorVelSub = this->create_subscription<std_msgs::msg::Float32>(
        "topic", 10, std::bind(&SerialCom::topicCallback, this, std::placeholders::_1));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCom>());
    rclcpp::shutdown();

    return 0;
}
