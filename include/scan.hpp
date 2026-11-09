#ifndef _SCAN_HPP_
#define _SCAN_HPP_

#include "boost/lockfree/spsc_queue.hpp"
#include "Eigen/Dense"
#include <memory>

#include "sensor/laser.hpp"
#include "base_struct.hpp"
#include "sensor/imu.hpp"
namespace scout
{
class scan
{
public:

    scan(rclcpp::executors::SingleThreadedExecutor::SharedPtr exec,Options* option);
    void laser_point_to_world_point(sense::Pose_anglef laser_world,float angle_local,float range);

    void use_laser_node(std::string name = "laser_node", std::string topic = "scan")
    {
        laser_node = std::make_shared<sense::Laser_node>(name, point_cloud_data_, topic);
        exec_->add_node(laser_node);
    }

    void scan_build_front();//前端建图
float angle_min;                // 这一帧扫描的起始角度（弧度，通常是 -π 或 0）
float angle_max;                // 这一帧扫描的结束角度（弧度，通常是 π 或 2π）
float scan_time;                // 扫完一圈/一帧的总时间（秒）
float range_min;                // 传感器能测到的最小有效距离（米，比如 0.1）
float range_max;                // 传感器能测到的最大有效距离（米，比如 12.0）
float time_increment;           // 相邻两个数据点之间的时间增量（秒）
float angle_increment;          // 相邻两个数据点之间的角度增量（弧度）
float laser_offset_x;//雷达相对于base_link的偏移
float laser_offset_y;//雷达相对于base_link的偏移
float laser_offset_angle;//雷达相对于base_link的偏移

private :
    bool scan_first=0;
    Options* options_;

    std::shared_ptr<boost::lockfree::spsc_queue<
    sensor_msgs::msg::LaserScan::SharedPtr, 
    boost::lockfree::capacity<10>
    >> point_cloud_data_;//点云无锁队列  shared_ptr方便移动数据 点云太大了 呃但是move以后还能pop吗
    std::vector<Eigen::Vector2f> point_cloud_temp_;

    std::shared_ptr<sense::Laser_node> laser_node;

    sense::pose_extrapolator extrapolator_{exec_,options_};//姿态外推器 在这里获取多线程执行器
    std::vector<sense::Pose_anglef>pose_extra_temp;
    sense::Pose_time pose_best_;

    rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;  //多线程接口  
};

}//namespace
#endif