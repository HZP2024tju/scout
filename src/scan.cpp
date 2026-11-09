#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "Eigen/Dense"

#include "sensor/filter.hpp"
#include "scan.hpp"
#include "settings.hpp"
namespace scout
{//建图前端
scan::scan(rclcpp::executors::SingleThreadedExecutor::SharedPtr exec,Options *option):exec_(exec),options_(option),
 laser_offset_x(option->laser_offset_x),laser_offset_y(option->laser_offset_y),laser_offset_angle(option->laser_offset_angle)
{
    point_cloud_temp_.reserve(point_cloud_size);
    pose_extra_temp.reserve(point_cloud_size);
    extrapolator_.use_imu_node();
    extrapolator_.use_odom_node();//@TODO这里要根据options设置
}

scan::~scan()
{
}

void scan::laser_point_to_world_point(sense::Pose_anglef laser_world,float angle_local,float range)
{//计算从局部到全局的点

}

void scan::scan_build_front()
{
    if (!point_cloud_data_->read_available())
    {//没数据就返回
        return;
    }

    if (!scan_first)
    {//如果是第一次
        pose_best_.pose = Eigen::Vector2f(0,0);
        pose_best_.yaw = 0;
        pose_best_.stamp = laser_node->now();
        extrapolator_.add_pose(pose_best_.pose,pose_best_.yaw,pose_best_.stamp);
        angle_min = point_cloud_data_->front()->angle_min;                // 这一帧扫描的起始角度（弧度，通常是 -π 或 0）一般都认为是0吧 这个待测 我们将这个作为修正量
        angle_max = point_cloud_data_->front()->angle_max;  
        scan_time = point_cloud_data_->front()->scan_time; 
        range_min = point_cloud_data_->front()->range_min; 
        range_max = point_cloud_data_->front()->range_max; 
        scan_first=1;
    }
    else
    {//不是第一次
        pose_best_ = extrapolator_.pose_data_.back();
    }
    
    sensor_msgs::msg::LaserScan::SharedPtr current_scan;

    float* range_arr;
    size_t point_count;
    double time_push_temp;
    sense::Pose_anglef pose_push_temp;

    while (point_cloud_data_->pop(current_scan)) 
    {//取出数据 直到没有为止
        time_increment=current_scan->time_increment;//看看本次点间隔多久
        angle_increment=current_scan->angle_increment;
        time_push_temp=pose_best_.stamp.seconds();
        pose_push_temp.pose=pose_best_.pose;            //推导起始
        pose_push_temp.yaw=pose_best_.yaw;    
        
        range_arr = current_scan->ranges.data();
        point_count = current_scan->ranges.size();//数据

        for (size_t i = 0; i < point_count; ++i) 
        {
            extrapolator_.extrapolator(time_push_temp,pose_push_temp);
            point_cloud_temp_.emplace_back(pose_push_temp);
            time_push_temp += time_increment;
            angle_increment += angle_increment;//这俩应该放前面好还是后面好
        }

    }
}

}