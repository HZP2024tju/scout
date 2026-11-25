#ifndef _SCOUT_HPP_
#define _SCOUT_HPP_
#include <vector>
#include <mutex>
#include <memory>
#include "boost/lockfree/spsc_queue.hpp"
#include "boost/thread/thread.hpp"
#include "rclcpp/rclcpp.hpp"

#include "submap.hpp"
#include "scan.hpp"
#include "sensor/imu.hpp"
#include "base_struct.hpp"
namespace scout
{

class MapBuilder 
{       //总建图调度器
public:

    MapBuilder(const Options options):options_(options)
    {//@todo 这里应该按options选择是否使用imu odom等 
        call_backs_ = std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        
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

    void build_front();
    void run()
    {   
        exec_thread_ = boost::thread([this]()
            {this->exec_->spin();});//分一个线程出去专门跑节点与回调 我们是事件驱动模型
        
        rclcpp::WallRate loop_rate(100); 

        while (rclcpp::ok()) 
        {
            scaner.scan_build_front();//运行前端建图
            loop_rate.sleep(); 
        }
    }
private:
    std::mutex map_mutex_; // 保护地图的锁
    Options options_;

    rclcpp::CallbackGroup::SharedPtr call_backs_;  //多线程回调函数组
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_; //嵌入式平台（树莓派4b）性能有限，对于小开销的回调而言串行或许是更好的做法
    boost::thread exec_thread_;

    std::vector<std::unique_ptr<scout::ActiveSubmaps2D>> front_maps_; //两张前端活跃子图
    std::vector<std::unique_ptr<scout::Submap2D>> frozen_maps_; //已冻结子图

    scan scaner{exec_,&options_};
}; 

} // namespace scout
#endif