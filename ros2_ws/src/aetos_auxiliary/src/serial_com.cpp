#include <rclcpp/rclcpp.hpp>
#include <aetos_msgs/msg/velocity.hpp>

#include <libudev.h>
#include <boost/asio.hpp>
#include <iostream>
#include <string>

class SerialCom : public rclcpp::Node
{
public:
    SerialCom();

private:
    void topicCallback(const aetos_msgs::msg::Velocity::SharedPtr msg);
    std::string usbMonitorCallback();
    // void usbMonitorCallback(const std::string &port);

    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial;

    std::string _serialPort;
    std::thread _usbThread;
    bool _serialConnected;

    rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr _motorVelSub;
    rclcpp::TimerBase::ConstSharedPtr _usbMonitorTimer;
};

void SerialCom::topicCallback(const aetos_msgs::msg::Velocity::SharedPtr msg)
{
    try
    {
        boost::asio::write(_serial, boost::asio::buffer(&msg->data[msg->VX], sizeof(float)));
        boost::asio::write(_serial, boost::asio::buffer(&msg->data[msg->VY], sizeof(float)));
        boost::asio::write(_serial, boost::asio::buffer(&msg->data[msg->VZ], sizeof(float)));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
    }
}

std::string SerialCom::usbMonitorCallback()
{
    struct udev *udev = udev_new();

    if (!udev)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize udev.");
        return "";
    }

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices, *dev_list_entry;
    devices = udev_enumerate_get_list_entry(enumerate);

    std::string detected_port = "";

    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);
        if (dev)
        {
            const char *devnode = udev_device_get_devnode(dev);
            if (devnode && (std::string(devnode).find("/dev/ttyUSB") != std::string::npos ||
                            std::string(devnode).find("/dev/ttyACM") != std::string::npos))
            {
                detected_port = devnode;
                udev_device_unref(dev);
                break;
            }
            udev_device_unref(dev);
        }
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return detected_port;
}

SerialCom::SerialCom() : Node("serial_comm"), _serial(_io_service)
{
    _motorVelSub = this->create_subscription<aetos_msgs::msg::Velocity>(
        "aetos/control/motor_speed", 10, std::bind(&SerialCom::topicCallback, this, std::placeholders::_1));

    _usbMonitorTimer = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&SerialCom::usbMonitorCallback, this));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCom>());
    rclcpp::shutdown();

    return 0;
}
