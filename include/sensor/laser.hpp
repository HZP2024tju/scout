#ifndef _LASER_HPP_
#define _LASER_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "boost/lockfree/spsc_queue.hpp"
#include "Eigen/Dense"
#include <memory>

#include "base_struct.hpp"
namespace sense{
class Laser_node : public rclcpp::Node
{
public:
    Laser_node(std::string name="laser_node", std::shared_ptr<boost::lockfree::spsc_queue<
    sensor_msgs::msg::LaserScan::SharedPtr, 
    boost::lockfree::capacity<10>
    >> point_cloud_data,std::string topic = "scan");
    
private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan );
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_; //SharePtr便于管理

    std::shared_ptr<boost::lockfree::spsc_queue<
    sensor_msgs::msg::LaserScan::SharedPtr, 
    boost::lockfree::capacity<10>
    >> point_cloud_data_;//无锁队列
};

}
#endif