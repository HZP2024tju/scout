#ifndef _SCOUT_HPP_
#define _SCOUT_HPP_
#include <vector>
#include <mutex>
#include "submap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "imu.hpp"
#include <thread>
namespace scout
{
class MapBuilder 
{       //总建图调度器
public:
    // 提供安全的数据访问接口
    void AddFrontMap(std::unique_ptr<scout::ActiveSubmaps2D> map) 
    {
        std::lock_guard<std::mutex> lock(map_mutex_); // 加锁
        front_maps_.emplace_back(std::move(map));
    }
    
    // 记得返回的时候也要处理锁和引用传递
    std::vector<std::unique_ptr<scout::ActiveSubmaps2D>>& GetFrontMaps() {
        return front_maps_; // 如果这里返回引用，外部遍历时也必须加锁，或者返回拷贝
    }
    

private:
    std::mutex map_mutex_; // 保护地图的锁
    
    std::vector<std::unique_ptr<scout::ActiveSubmaps2D>> front_maps_; //两张前端活跃子图
    std::vector<std::unique_ptr<scout::Submap2D>> frozen_maps_; //已冻结子图

    sensor::pose_extrapolator extrapolator_;
}; 
} // namespace scout
#endif