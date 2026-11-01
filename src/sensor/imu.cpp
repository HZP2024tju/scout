#include "imu.hpp"

namespace sensor
{
    //构造函数
Imu_node::Imu_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<float>>quene ,std::string topic = "imu")
        :Node(name),quene_(quene)
{
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(topic,10,
    [this](sensor_msgs::msg::Imu::SharedPtr imu_data){this->imu_callback(imu_data);} ); //传入的回调函数必须是一个实例
}

void Imu_node::imu_callback(const sensor_msgs::msg::Imu::SharedPtr& imu_data ) //ros2习惯写法
{//话题回调函数   基于标准消息
    quene_->push(imu_data->angular_velocity.z);//仅推入z轴数据
}

odom_node::odom_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<Eigen::Vector2f>>quene ,std::string topic = "odom")
        :Node(name),quene_(quene)
{
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(topic,10,
    [this](nav_msgs::msg::Odometry::SharedPtr odom_data){this->odom_callback(odom_data);} ); //传入的回调函数必须是一个实例
};

void odom_node::odom_callback(const nav_msgs::msg::Odometry::SharedPtr& odom_data ) //ros2习惯写法
{//话题回调函数   基于标准消息
    quene_->push(Eigen::Vector2f(odom_data->pose.pose.position.x,odom_data->pose.pose.position.y) );//推入里程计推导的位姿数据
}
}