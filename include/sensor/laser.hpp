#ifndef _LASER_HPP_
#define _LASER_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace sense{
class Laser_node : public rclcpp::Node
{
public:
    Laser_node(std::string name,std::string topic = "scan"): Node(name);
    
private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr& scan );

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_; //SharePtr便于管理
};

}
#endif