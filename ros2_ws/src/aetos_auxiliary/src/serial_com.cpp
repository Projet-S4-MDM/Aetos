#include <rclcpp/rclcpp.hpp>
#include <aetos_msgs/msg/motor_velocity.hpp>
#include <aetos_msgs/msg/encoder_values.hpp>

#include <libudev.h>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <mutex>
#include <libserial/SerialPort.h>

constexpr uint16_t USB_MONITOR_FREQ = 500;   // 2Hz
constexpr uint16_t SERIAL_MONITOR_FREQ = 10; // 100 hZ
constexpr uint32_t BAUD_RATE = 115200; // 100 hZ

class SerialCom : public rclcpp::Node
{
public:
    SerialCom();
    ~SerialCom();

private:
    void velocityMsgCallbcak(const aetos_msgs::msg::MotorVelocity::SharedPtr velocityMsg);
    void serialMonitor(void);
    void usbMonitor(void);
    std::string findSerialPort(void);
    bool openSerialPort(const std::string &port);
    void closeSerialPort(void);

    bool _serialConnected;
    std::string _serialPort;
    std::mutex _serialMutex;
    std::thread _ioThread;

    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial;

    rclcpp::TimerBase::SharedPtr _serialMonitorTimer;
    rclcpp::TimerBase::SharedPtr _usbMonitorTimer;

    rclcpp::Subscription<aetos_msgs::msg::MotorVelocity>::SharedPtr _motorVelSub;
    rclcpp::Publisher<aetos_msgs::msg::EncoderValues>::SharedPtr _encoderPub;
};

void SerialCom::velocityMsgCallbcak(const aetos_msgs::msg::MotorVelocity::SharedPtr velocityMsg_)
{
    std::lock_guard<std::mutex> lock(_serialMutex);
    if (!_serialConnected)
    {
        RCLCPP_WARN(this->get_logger(), "Serial port not connected. Dropping message.");
        return;
    }
    try
    {
        boost::asio::write(_serial, boost::asio::buffer(&velocityMsg_->data[velocityMsg_->W_1], sizeof(float)));
        boost::asio::write(_serial, boost::asio::buffer(&velocityMsg_->data[velocityMsg_->W_2], sizeof(float)));
        boost::asio::write(_serial, boost::asio::buffer(&velocityMsg_->data[velocityMsg_->W_3], sizeof(float)));
        boost::asio::write(_serial, boost::asio::buffer(&velocityMsg_->data[velocityMsg_->W_4], sizeof(float)));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
    }
}

void SerialCom::closeSerialPort(void)
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

bool SerialCom::openSerialPort(const std::string &port_)
{
    try
    {
        _serial.open(port_);
        _serial.set_option(boost::asio::serial_port_base::baud_rate(BAUD_RATE));
        _serial.set_option(boost::asio::serial_port_base::character_size(8));
        _serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        _serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        _serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        RCLCPP_INFO(this->get_logger(), "Opened port: %s", port_.c_str());
        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        return false;
    }
}

void SerialCom::serialMonitor(void)
{
    std::lock_guard<std::mutex> lock(_serialMutex);
    if (!_serialConnected)
    {
        return;
    }

    try
    {
        std::array<char, sizeof(float) * 4> buffer;
        boost::system::error_code error;
        size_t bytesRead = 0;

        while (bytesRead < sizeof(float) * 4)
        {
            size_t bytesTransfered = _serial.read_some(boost::asio::buffer(buffer.data() + bytesRead, buffer.size() - bytesRead), error);
            bytesRead += bytesTransfered;

            if (error && error != boost::asio::error::would_block)
            {
                throw boost::system::system_error(error);
            }
        }

        if (bytesRead == sizeof(float) * 4)
        {
            aetos_msgs::msg::EncoderValues encoderMsg;
            std::memcpy(&encoderMsg.data[encoderMsg.ANGLE_1], &buffer[0 * sizeof(float)], sizeof(float));
            std::memcpy(&encoderMsg.data[encoderMsg.ANGLE_2], &buffer[1 * sizeof(float)], sizeof(float));
            std::memcpy(&encoderMsg.data[encoderMsg.ANGLE_3], &buffer[2 * sizeof(float)], sizeof(float));
            std::memcpy(&encoderMsg.data[encoderMsg.ANGLE_4], &buffer[3 * sizeof(float)], sizeof(float));

            _encoderPub->publish(encoderMsg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received unexpected number of bytes: %zu", bytesRead);
        }
    }
    catch (const boost::system::system_error &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        _serialConnected = false;
    }
}

void SerialCom::usbMonitor(void)
{
    std::string detectedPort = findSerialPort();

    if (detectedPort.empty())
    {
        if (_serialConnected)
        {
            RCLCPP_WARN(this->get_logger(), "Serial port disconnected");
            closeSerialPort();
        }
        return;
    }

    if (!_serialConnected || _serialPort != detectedPort)
    {
        std::lock_guard<std::mutex> lock(_serialMutex);
        closeSerialPort();
        if (openSerialPort(detectedPort))
        {
            _serialPort = detectedPort;
            _serialConnected = true;
            RCLCPP_INFO(this->get_logger(), "Connected to %s", detectedPort.c_str());
        }
    }
}

std::string SerialCom::findSerialPort(void)
{
    struct udev *udev = udev_new();
    if (!udev)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize udev.");
        return "";
    }

    std::string detectedPort;
    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    // These stucts are native to the udev libs, thus why they do not comply with naming conventions
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
                detectedPort = devnode;
                udev_device_unref(dev);
                break;
            }
            udev_device_unref(dev);
        }
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    return detectedPort;
}

SerialCom::SerialCom() : Node("serial_comm"), _serial(_io_service)
{
    _motorVelSub = this->create_subscription<aetos_msgs::msg::MotorVelocity>(
        "aetos/control/motor_speed", 10,
        std::bind(&SerialCom::velocityMsgCallbcak, this, std::placeholders::_1));

    _encoderPub = this->create_publisher<aetos_msgs::msg::EncoderValues>(
        "aetos/control/encoder", 1);

    _usbMonitorTimer = this->create_wall_timer(
        std::chrono::milliseconds(USB_MONITOR_FREQ),
        std::bind(&SerialCom::usbMonitor, this));

    _serialMonitorTimer = this->create_wall_timer(
        std::chrono::milliseconds(SERIAL_MONITOR_FREQ),
        std::bind(&SerialCom::serialMonitor, this));

    _ioThread = std::thread([this]()
        {
            while (rclcpp::ok()) 
            {
                try 
                {
                    _io_service.poll();
                    _io_service.reset();
                } 
                catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "IO Service error: %s", e.what());
                }
            }
        });
}

SerialCom::~SerialCom()
{
    closeSerialPort();
    _io_service.stop();

    if (_ioThread.joinable())
    {
        _ioThread.join();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCom>());
    rclcpp::shutdown();

    return 0;
}
