#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "laser.hpp"
#include "imu.hpp"
int main(int argc,char**argv)
{
    //std::thread ros_thread([pub]()//创建一个线程运行ros 最好是低占用率
    rclcpp::init(argc,argv);
    rclcpp::shutdown();
}