#ifndef _SCOUT_HPP_
#define _SCOUT_HPP_
#include <vector>
#include <mutex>
#include "submap.hpp"
#include <memory>
#include "boost/lockfree/spsc_queue.hpp"
#include "boost/thread/thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "imu.hpp"

namespace scout
{
struct Pose_time 
{
rclcpp::Time stamp;
Eigen::Vector2f pose;
float yaw;
Pose_time( rclcpp::Time& t,  Eigen::Vector2f& d,float& yaw_in) 
        : stamp(t), pose(d) ,yaw(yaw_in) {}
};

struct Pose_anglef
{
Eigen::Vector2f pose;
float yaw;
Pose_anglef(  Eigen::Vector2f& d,float &yaw_in) 
        : pose(d),yaw(yaw_in) {}
};
class MapBuilder 
{       //总建图调度器
public:

    MapBuilder(const std::vector<int>& options):options_(options)
    {//@todo 这里应该按options选择是否使用imu odom等 
        call_backs_ = std::make_shared<rclcpp::CallbackGroup>();
        exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        
    }
      
    ~MapBuilder() {
        // 先让 ROS 2 关闭 退出阻塞 
        exec_->cancel();
        rclcpp::shutdown();
        //等待后台线程真正执行完毕
        if (exec_thread_.joinable()) {
            exec_thread_.join();
        }
    }

    void run()
    {
        
        exec_thread_ = boost::thread([this]()
            {this->exec_->spin();});
    }
private:
    std::mutex map_mutex_; // 保护地图的锁
    std::vector<int> options_;

    rclcpp::CallbackGroup::SharedPtr call_backs_;  //多线程回调函数组
    rclcpp::executors::MultiThreadedExecutor::SharedPtr exec_;  //多线程接口
    boost::thread exec_thread_;

    std::vector<std::unique_ptr<scout::ActiveSubmaps2D>> front_maps_; //两张前端活跃子图
    std::vector<std::unique_ptr<scout::Submap2D>> frozen_maps_; //已冻结子图

    sense::pose_extrapolator extrapolator_{exec_,call_backs_};
}; 

} // namespace scout
#endif