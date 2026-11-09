#ifndef _BASE_STRUCT_HPP_
#define _BASE_STRUCT_HPP_
#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include <vector>
//为了防止循环嵌套出的下策
namespace sense
{
struct Pose_time 
{
rclcpp::Time stamp;
Eigen::Vector2f pose;
float yaw;

Pose_time( rclcpp::Time& t,  Eigen::Vector2f& d,float& yaw_in) 
        : stamp(t), pose(d) ,yaw(yaw_in) {}
Pose_time() = default;
};

struct Pose_anglef
{
Eigen::Vector2f pose;
float yaw;
Pose_anglef(  Eigen::Vector2f& d,float &yaw_in) 
        : pose(d),yaw(yaw_in) {}
Pose_anglef() = default;
};

struct Scan_point_Msg
{
    float time_increment;//与上一个点的间隔
    float range;//距离
Scan_point_Msg( float& t,float& range_in) 
        :time_increment(t),range(range_in){}
};

}

namespace scout
{
    //记录选项的结构体 向下传递
struct Options
{
    int vox_lenth;//体素滤波长  一般都是 1/分辨率
    double pose_trust_time;
    double odom_trust_time;
    double imu_trust_time;
    float laser_offset_x;//雷达相对于base_link的偏移
    float laser_offset_y;//雷达相对于base_link的偏移
    float laser_offset_angle;//雷达相对于base_link的偏移
};
}
#endif