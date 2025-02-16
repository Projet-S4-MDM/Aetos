#include <rclcpp/rclcpp.hpp>
#include <aetos_msgs/msg/velocity.hpp>
#include <libudev.h>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <mutex>

class SerialCom : public rclcpp::Node
{
public:
    SerialCom() : Node("serial_comm"), _serial(_io_service), _serialConnected(false)
    {
        _motorVelSub = this->create_subscription<aetos_msgs::msg::Velocity>(
            "aetos/control/motor_speed", 10,
            std::bind(&SerialCom::topicCallback, this, std::placeholders::_1));

        _usbMonitorTimer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SerialCom::usbMonitorCallback, this));

        _ioThread = std::thread([this]()
                                {
            while (rclcpp::ok()) {
                try {
                    _io_service.poll();
                    _io_service.reset();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "IO Service error: %s", e.what());
                }
            } });
    }

    ~SerialCom()
    {
        closeSerialPort();
        _io_service.stop();
        if (_ioThread.joinable())
        {
            _ioThread.join();
        }
    }

private:
    void topicCallback(const aetos_msgs::msg::Velocity::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_serialMutex);
        if (!_serialConnected)
        {
            RCLCPP_WARN(this->get_logger(), "Serial port not connected. Dropping message.");
            return;
        }

        try
        {
            // Format the data as a string with newline
            std::string data = "VX:" + std::to_string(msg->data[msg->VX]) +
                               " VY:" + std::to_string(msg->data[msg->VY]) +
                               " VZ:" + std::to_string(msg->data[msg->VZ]) + "\n";

            boost::asio::write(_serial, boost::asio::buffer(data));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
            _serialConnected = false; // Mark as disconnected to trigger reconnection
        }
    }

    void usbMonitorCallback()
    {
        std::string detected_port = findSerialPort();

        if (detected_port.empty())
        {
            if (_serialConnected)
            {
                RCLCPP_WARN(this->get_logger(), "Serial port disconnected");
                closeSerialPort();
            }
            return;
        }

        if (!_serialConnected || _serialPort != detected_port)
        {
            std::lock_guard<std::mutex> lock(_serialMutex);
            closeSerialPort();
            if (openSerialPort(detected_port))
            {
                _serialPort = detected_port;
                _serialConnected = true;
                RCLCPP_INFO(this->get_logger(), "Connected to %s", detected_port.c_str());
            }
        }
    }

    bool openSerialPort(const std::string &port)
    {
        try
        {
            _serial.open(port);
            _serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
            _serial.set_option(boost::asio::serial_port_base::character_size(8));
            _serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            _serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            _serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            return false;
        }
    }

    void closeSerialPort()
    {
        if (_serial.is_open())
        {
            try
            {
                _serial.close();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error closing serial port: %s", e.what());
            }
        }
        _serialConnected = false;
    }

    std::string findSerialPort()
    {
        struct udev *udev = udev_new();
        if (!udev)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize udev.");
            return "";
        }

        std::string detected_port;
        struct udev_enumerate *enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, "tty");
        udev_enumerate_scan_devices(enumerate);

        struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry *dev_list_entry;

        for (dev_list_entry = devices; dev_list_entry != nullptr;
             dev_list_entry = udev_list_entry_get_next(dev_list_entry))
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

    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial;
    std::string _serialPort;
    bool _serialConnected;
    std::mutex _serialMutex;
    std::thread _ioThread;

    rclcpp::Subscription<aetos_msgs::msg::Velocity>::SharedPtr _motorVelSub;
    rclcpp::TimerBase::SharedPtr _usbMonitorTimer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCom>());
    rclcpp::shutdown();
    return 0;
}