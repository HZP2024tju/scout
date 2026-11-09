#include "laser.hpp"
#include "settings.hpp"
//#include "absl/hash/hash.h"  //absl20211102 可选地使用absl的哈希表吧 但是掺进太多三方库不太好
//制造节点可供使用的传感器层与滤波器
namespace sense
{
//构造函数
//std::accumulate
Laser_node::Laser_node(std::string name="laser_node",std::shared_ptr<boost::lockfree::spsc_queue<
    sensor_msgs::msg::LaserScan::SharedPtr, 
    boost::lockfree::capacity<10>
>> point_cloud_data,std::string topic = "scan"):Node(name),point_cloud_data_(point_cloud_data) //从这个构造函数继承
{
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic,10,
    [this](sensor_msgs::msg::LaserScan::SharedPtr scan){this->laser_callback(scan);} ); //传入的回调函数必须是一个实例
}

void Laser_node::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan ) //ros2习惯写法
{//话题回调函数   基于标准消息
        //调用滤波器
        //调用前端
    point_cloud_data_->push(std::move(scan));
}


}//namespace