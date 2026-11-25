#ifndef _BASE_STRUCT_HPP_
#define _BASE_STRUCT_HPP_
#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include <vector>
//为了防止循环嵌套出的下策
namespace scout
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
        : pose(d),yaw(yaw_in) {};

Pose_anglef( const float x,const float y,const float &yaw_in) 
        : pose{x,y}  ,yaw(yaw_in) {};
Pose_anglef() = default;
};

struct Scan_point_Msg
{
    float time_increment;//与上一个点的间隔
    float range;//距离
Scan_point_Msg( float& t,float& range_in) 
        :time_increment(t),range(range_in){}
};

struct avoid_thread_scan_score_fake_share_
{//填充48字节
    uint8_t a[48];
};
    //记录选项的结构体 向下传递
struct Options
{
    int vox_lenth;//体素滤波长  一般都是 1/分辨率
    double pose_trust_time;
    double odom_trust_time;
    double imu_trust_time;
    float laser_offset_x;//雷达相对于base_link的偏移
    float laser_offset_y;//雷达相对于base_link的偏移
    float laser_offset_angle;//雷达相对于base_link的偏移 我们是右手系逆时针为正
    bool  laser_follow_clock;//雷达是顺时针还是逆时针 1(true)为逆时针
    float laser_start;//雷达是顺时针还是逆时针 1(true)为逆时针 设置扫描起步，比如如果是360°顺时针，这里就是2pi
    uint16_t laser_hz;//雷达扫描频率
    uint32_t point_cloud_size ;//360  //请在运行前确定 雷达单次扫描最大点云数
    bool see_map  ;//是否发布地图  0就是只会在结束时发一次地图并记录 1
    bool use_particle_scan; //是否使用粒子滤波粗匹配
    float angle_search_window;//醋扫的角度窗口 最高是360°
    float linear_search_window;//粗扫的线性窗口
    float search_step_linear;//线性搜索步长
    float angle_cost_weight_;
    float linear_cost_weight_;//粗搜索的时候相对原点移动的损耗
    float submap_lenth;
    float submap_wide;
    float resolution;//地图分辨率
    uint16_t insert_max_;//前端活跃子图最多插入扫描数
    double Rotation_weight;
    double Translate_weight;//前端旋转和平移优化的权重
    double Front_ceres_weight;//前端旋转和平移优化的权重
    float ceres_max_time;
    float ceres_max_step;
};
}
#endif