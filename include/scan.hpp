#ifndef _SCAN_HPP_
#define _SCAN_HPP_

#include "boost/lockfree/spsc_queue.hpp"
#include "Eigen/Dense"
#include <memory>
#include "ceres/ceres.h"
#include "ceres/jet.h"

#include "sensor/laser.hpp"
#include "base_struct.hpp"
#include "sensor/imu.hpp"
#include "sensor/filter.hpp"
#include "pose_graph.hpp"
#include "submap.hpp"
namespace scout
{
class scan
{
public:

    scan(rclcpp::executors::SingleThreadedExecutor::SharedPtr exec,Options* option);

    void use_laser_node(std::string name = "laser_node", std::string topic = "scan")
    {
        laser_node_ = std::make_shared<sense::Laser_node>(point_cloud_data_,name, topic);
        exec_->add_node(laser_node_);
    }

    void scan_build_front();//前端建图

private :
    struct alignas(64) ThreadBestResult 
    {
        float score;
        Pose_anglef pose;
        // 让编译器自动帮我们填充剩余空间到 64 字节来防止伪共享
    };
    std::unique_ptr<std::vector<Eigen::Vector2f>> fix_pointcloud_with_imu(float* range_arr ,size_t& point_count, Pose_anglef& pose_push,double time_base);

    float compute_pointcloud_score(std::vector<Eigen::Vector2f>&point_cloud) ;
    void scan_particle_match(Pose_anglef& pose_origin, std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_ptr);
    void scan_ceres_match(Pose_anglef& pose_origin, std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_ptr);
    ActiveSubmaps2D active_submaps_;

    float angle_min;                // 这一帧扫描的起始角度（弧度，通常是 -π 或 0）
    float angle_max;                // 这一帧扫描的结束角度（弧度，通常是 π 或 2π）
    float scan_time;                // 扫完一圈/一帧的总时间（秒）
    float range_min;                // 传感器能测到的最小有效距离（米，比如 0.1）
    float range_max;                // 传感器能测到的最大有效距离（米，比如 12.0）
    float time_increment;           // 相邻两个数据点之间的时间增量（秒）
    float angle_increment;          // 相邻两个数据点之间的角度增量（弧度）
    float laser_offset_x;//雷达相对于base_link的偏移            我们规定右手系而且逆时针为正，逆时针扫描的情况下雷达一点是从0开始扫描,否则就从2pi开始反向扫描
    float laser_offset_y;//雷达相对于base_link的偏移
    float laser_offset_angle;//雷达相对于base_link的偏移 我们是右手系逆时针为正
    bool  laser_follow_clock;//雷达是顺时针还是逆时针 1(true)为逆时针
    float laser_start;//设置扫描起步，比如如果是360°顺时针，这里就是2pi  逆时针就是0
    uint32_t point_cloud_size;

    bool scan_first=0;//扫描开启
    float resolution_;//分辨率
    float angle_search_window;//粗扫的角度窗口 最高是360°
    float linear_search_window;//粗扫的线性窗口
    float search_step_linear;//线搜索步长
    double angle_cost_weight_;//一般取 10@TODO看看调参效果
    double translate_cost_weight_;//线性搜索惩罚权重 40
    double Front_ceres_weight;

    float ceres_max_time;
    float ceres_max_step;
    bool use_particle_scan;//使用粒子滤波
    bool use_fix_pointcloud_with_imu;//使用体素滤波
    Options* options_;

    std::shared_ptr<boost::lockfree::spsc_queue<
    sensor_msgs::msg::LaserScan::SharedPtr, 
    boost::lockfree::capacity<10>
    >> point_cloud_data_;//点云无锁队列  shared_ptr方便移动数据 点云太大了 呃但是move以后还能pop吗
    sense::filters filter_{options};

    std::shared_ptr<sense::Laser_node> laser_node_;

    sense::pose_extrapolator extrapolator_{exec_,options_};//姿态外推器 在这里获取多线程执行器    

    std::shared_ptr<boost::lockfree::spsc_queue<
    graph::constraint, 
    boost::lockfree::capacity<30>
    >> constraint_cloud_data_;//无锁队列 向后端传输约束

    std::shared_ptr<boost::lockfree::spsc_queue<
    graph::submap_back, 
    boost::lockfree::capacity<30>
    >> node_data_;//无锁队列 向后端传输节点

    std::shared_ptr<boost::lockfree::spsc_queue<
    graph::node_scan, 
    boost::lockfree::capacity<30>
    >> submap_data_;//无锁队列 向后端传输已冻结子图数据

    rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;  //多线程接口  
};


class Front_Cost_Function2D {
 public:
Front_Cost_Function2D(const Submap2D* submap,
                    const std::unique_ptr<std::vector<Eigen::Vector2f>>& global_point_cloud_ptr,const Pose_anglef& pose_origin,double weight);//这里我们选择使用子图传过来grid

template <typename T>
bool operator()(const T* const pose, T* residual) const 
{
    //这是一个计算残差的函数
    Eigen::Matrix<T, 2, 1> pose_diff= Eigen::Matrix<T, 2, 1>
                    { -  T(pose_origin_.pose.x()),
                    -  T(pose_origin_.pose.y()) } ;
    Eigen::Matrix<T, 2, 1> pose_diff_submap = Eigen::Matrix<T, 2, 1>{ pose[0] - T(submap_ptr_->origin.pose.x()) ,pose[1] - T(submap_ptr_->origin.pose.y())};
    Eigen::Rotation2D<T> roation_pose( pose[2] - T(pose_origin_.yaw) );//将全局点云变化到当前位姿       这里是绕0，0转
    Eigen::Rotation2D<T> roation_submap = Eigen::Rotation2D<T>(T(submap_ptr_->origin.yaw)).inverse();//全局转到子图的局部旋转矩阵
    Eigen::Matrix<T, 2, 1> point;  
    for (size_t i = 0; i < point_cloud_.size(); ++i) 
    {                                      
        point =  roation_submap *  ( roation_pose * (point_cloud_[i]+pose_diff )  + pose_diff_submap);
        residual[i] =  (T(1.0) - submap_ptr_->GetlocalLinearFloatProbability(point(0),point(1))) * T(weight_);//计算残差
        
    }
    return true;
}

 private:
  static constexpr int kPadding = INT_MAX / 4;
  
  const std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_;
  const Submap2D* submap_ptr_;
  const Pose_anglef& pose_origin_;
  const double weight_;
};
              //自动选用了动态    默认为QR求解        @TODO从自定的块中获取

class RotationDeltaCostFunctor2D
{
 public:
    RotationDeltaCostFunctor2D(float& angle_origin,const float& wight) : wight_(wight),angle_(angle_origin)
    {};

  template <typename T>
  bool operator()(const T* const pose, T* residual) const
    {
        residual[0] = wight_ * (pose[2] - angle_);
        return true;
    }
private:
    const float& wight_;
    const float& angle_;
};

class TranslationDeltaCostFunctor2D
{
 public:
    TranslationDeltaCostFunctor2D(float& x,float& y,const float& wight) : wight_(wight),x_(x),y_(y)
    {};

   template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = wight_ * (pose[0] - x_);
    residual[1] = wight_ * (pose[1] - y_);
    return true;
  }
private:
    const float& wight_;
    const float& x_;
    const float& y_;
};

}//namespace
#endif