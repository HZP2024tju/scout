#include <assert.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "boost/thread/thread.hpp"
#include "boost/asio/error.hpp"
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include "boost/asio/read.hpp"
#define recv_buff_size 100
#define port_baud_rate 115200
#define recv_lenth 14 //包长
#define recv_head 0xA5  //包头
#define recv_end  0xC1 //包尾
static std::string port_name ="/dev/ttyUSB0";
class port :public rclcpp::Node
{
public:
//自动读取数据并发布话题
    port(std::string name,std::string topic_imu="imu",std::string topic_odom="odom"):Node(name),serial_port_(io_service_)
    {
        frame_buffer_.reserve(recv_buff_size);//缓冲区
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_imu, 10);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom, 10);
        //串口初始化       这里是拿鱼香ros的改了点拿来用,他写得很好
            //说是要先打开串口再设置参数
        boost::system::error_code ec;
        serial_port_.open(port_name, ec);
        if (!serial_port_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(),"错误,串口未打开");
            return;
        }    
        assert(!ec);

        serial_port_.set_option(
            boost::asio::serial_port_base::baud_rate(port_baud_rate));  //波特率 
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));//8位数据位
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));                    //1位停止位
        serial_port_.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));              //无校验位。
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));            //无硬件流控  
    }
    ~port()
    {
        io_service_.stop();
        if (td.joinable()) {
            td.join();
        }
        if(serial_port_.is_open())
        {
            serial_port_.cancel();
            serial_port_.close();
        }
    }
    void read_data()
    {   
        serial_port_.async_read_some(boost::asio::buffer(recv_buffer_,recv_buff_size),[this](const boost::system::error_code& ec, std::size_t bytes)
        {read_callback(ec,bytes);});//不定长接收
    }
    void port_start()
    {
        read_data();
        td=boost::thread([this](){ this->io_service_.run();}); 
    }
private:
    void read_callback(const boost::system::error_code& error, std::size_t bytes_transferred)
    {//  @TODO 这里后面要加crc16检查 防止数据爆炸
        //这里的耗时不会很久，在回调里进行应该没问题？      不定长读取
        int offset_index= -frame_buffer_.size();  //记录包头相对于i的位置
        frame_buffer_.insert(frame_buffer_.end(),recv_buffer_,recv_buffer_+bytes_transferred);//数据追加 进缓冲区
        for (int i= 0; i<bytes_transferred;++i)
        {               //我们只遍历本次接收的数据
            if (frame_buf_step_==0 && frame_buffer_[i-offset_index] == recv_head)
            {
                frame_buf_step_=1;//希望获取队尾
                //删去多余数据
                frame_buffer_.erase(frame_buffer_.begin(),frame_buffer_.begin()+i-offset_index);
                offset_index=i;
            }
            else if (frame_buf_step_==1 && frame_buffer_[i-offset_index] == recv_end)
            {//真的在同一轮循环中找到了队尾   或者说，在下一次循环中找到了队尾 // 但是此时的数据已经没有了时效性 所以到底要不要进行拼接，本质上要看下位机程序的设定 我们这里还是选择拼接了
                //因为包截断发生的概率很小 而且基本上只会在高速链路中发生
                if( (i - offset_index+1)==recv_lenth )
                {//是消息
                    frame_buf_step_=0;//希望获取包头
                    recv_pub(frame_buffer_.data());//发布
                    frame_buffer_.erase(frame_buffer_.begin(),frame_buffer_.begin()+recv_lenth );//移除
                    offset_index=i+1;
                }
                else
                {
                    //并不是消息 删去多余数据  继续寻找下一个队头
                    frame_buf_step_=0;
                    frame_buffer_.erase(frame_buffer_.begin(),frame_buffer_.begin()+i-offset_index+1);
                    offset_index=i+1;
                }   
            } 
        }
        read_data();
    }//void      
    //必须要在reecv已经是完整数据的情况下调用
    void recv_pub(const uint8_t* frame_start)
    {
        msg_imu.header.stamp=this->now();
        msg_odom.header.stamp=this->now();
        std::memcpy(&msg_imu.angular_velocity.z,frame_start+1,sizeof(float));//imu z       
        std::memcpy(&msg_odom.pose.pose.position.x,frame_start+5,sizeof(float));//odom
        std::memcpy(&msg_odom.pose.pose.position.y,frame_start+9,sizeof(float));//odom     我们如此指定通信协议
        pub_imu_->publish(msg_imu);//发布
        pub_odom_->publish(msg_odom);//发布
    }
    uint8_t recv_buffer_[recv_buff_size];
    std::vector<uint8_t> frame_buffer_; //头部必然为包头
    bool frame_buf_step_=0; //0 表示接收包头,1表示需要接收结束包头
    std::vector<uint8_t>stm32_msg;
    boost::thread td;
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;

    nav_msgs::msg::Odometry msg_odom;
    sensor_msgs::msg::Imu   msg_imu;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;//odom发布
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;//imu发布
    rclcpp::Time time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<port>("port");
    node->port_start();//这里就已经进入串口读取循环了
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}