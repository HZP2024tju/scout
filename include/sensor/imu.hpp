#ifndef _IMU_HPP_
#define _IMU_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Dense"
#include "boost/lockfree/spsc_queue.hpp"
#include "boost/thread/thread.hpp"
#include <memory>
#include <queue>
#include <mutex>
namespace sensor{

class Imu_node : public rclcpp::Node
{
public:
    Imu_node::Imu_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<float>>quene ,std::string topic = "imu");
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr& imu_data );

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_; //SharePtr便于管理
    std::shared_ptr<boost::lockfree::spsc_queue<float>> quene_;//无锁队列 我们是单消费者单生产者情况,这样很快
};

class odom_node : public rclcpp::Node
{
public:
    odom_node::odom_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<Eigen::Vector2f>>quene ,std::string topic = "odom");
private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr& odom_data );
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_; //SharePtr便于管理
    std::shared_ptr<boost::lockfree::spsc_queue<Eigen::Vector2f> > quene_;//无锁队列 我们是单消费者单生产者情况,这样很快
};


class pose_extrapolator  //位姿推断器
{
public:

    pose_extrapolator()
    {
    }

    ~pose_extrapolator() {
        // 先让 ROS 2 关闭 退出阻塞 
        //等待后台线程真正执行完毕
        if (exec_thread_.joinable()) {
            exec_thread_.join();
        }
    }

   void use_odom_node(std::string name = "odom_node", std::string topic = "odom")
    {
        odom_node_ = std::make_shared<odom_node>(name, odom_data_, topic);
        exec_.add_node(odom_node_);
    }
    
    void use_imu_node(std::string name = "imu_node", std::string topic = "imu")
    {
        imu_node_ = std::make_shared<Imu_node>(name, angle_velocityer_, topic);
        exec_.add_node(imu_node_);
    }
    
    void node_run_all()
    {
        exec_thread_ = boost::thread([this]()
            {this->exec_.spin();});
    }
private:

    std::shared_ptr<Imu_node> imu_node_;
    std::shared_ptr<odom_node> odom_node_;

    boost::lockfree::spsc_queue<Eigen::Vector2f,boost::lockfree::capacity<50>> odom_data_; //空间后面写成define好一点 必须保证消费比生产快好一点 雷达10hz
    boost::lockfree::spsc_queue<float,boost::lockfree::capacity<50>> angle_velocityer_;// 里程计 imu传来的yaw轴角速度
    float linlinear_velocityer_from_odom_;//odom推断的线速度
    //float linlinear_velocityer_from_outside_;//下位机推导的线速度
    // std::queue<Eigen::Vector4f> angle_from_outside_;// 下位机推导的角度 四元数形式 节省计算
   // float imu_offset; //imu零飘

    rclcpp::CallbackGroup::SharedPtr call_backs_;  //多线程回调函数组
    rclcpp::executors::MultiThreadedExecutor exec_;  //多线程接口
    rclcpp::CallbackGroup::SharedPtr call_backs;//多线程回调
    boost::thread exec_thread_;
};

}
#endif