#include <assert.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "boost/thread/thread.hpp"
#include "boost/asio/error.hpp"
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#define recv_data_size 100
static std::string port_name ="/dev/ttyUSB0";
class port :public rclcpp::Node
{
public:
//自动读取数据并发布话题
    port(std::string name,std::string topic_imu="imu",std::string topic_odom="odom"):Node(name)
    {

    }
    void read_date()
    {
        serial_port_.async_read_some(boost::asio::buffer(recv_buffer_, recv_data_size),
        [this](const boost::system::error_code& ec, std::size_t bytes){read_callback(ec,bytes);}); 
    }

private:
    void init_port()
    {//串口初始化       这里是拿鱼香ros的改了点拿来用,他写得很好
        boost::system::error_code ec;
        serial_port_.open(port_name, ec);
        assert(!ec);
        serial_port_.set_option(
            boost::asio::serial_port_base::baud_rate(115200));
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));//8位数据位
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));                    //1位停止位
        serial_port_.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));              //无校验位。
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));            //无硬件流控
        read_date();
        td = boost::thread([this](){return this->io_service_.run});

    }

    int portDestory()
    {
        io_service_.stop();
        serial_port_.cancel();
        serial_port_.close();
        return 0;
    }
    void read_callback(const boost::system::error_code& error, std::size_t bytes_transferred)
    {
        if (bytes_transferred > 0)
        {
        std::string data_str(recv_data_buffer_, bytes_transferred);
        if (recv_data_callback_)
        {
            // 回调数据
            recv_data_callback_(data_str);
        }
        }
        _asyncReadSomeData();
    }
    char recv_buffer_[recv_data_size];
    boost::thread td;
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
    rclcpp::Publisher<nav_ms>::SharedPtr pub;
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<port>("port");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}