#ifndef _IMU_HPP_
#define _IMU_HPP_
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Dense"

#include "boost/lockfree/spsc_queue.hpp"
#include "boost/thread/thread.hpp"
#include <memory>
#include "scout.hpp"
namespace sense{

class Imu_node : public rclcpp::Node
{
public:
    struct ImuMsg
    {
        rclcpp::Time stamp;
        float angular_velocity;
        ImuMsg(const rclcpp::Time& t, const float& d) 
        : stamp(t), angular_velocity(d) {}
    };
    Imu_node::Imu_node(std::string name,std::shared_ptr< boost::lockfree::spsc_queue<ImuMsg,boost::lockfree::capacity<20>> >quene ,std::string topic = "imu");
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr& imu_data );

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_; //SharePtr便于管理
    std::shared_ptr<boost::lockfree::spsc_queue<ImuMsg,boost::lockfree::capacity<20>>> quene_;//维护一个无锁队列 我们是单消费者单生产者情况,这样很快

};

class odom_node : public rclcpp::Node
{
public:
    struct OdomMsg
    {
        rclcpp::Time stamp;//时间戳
        Eigen::Vector2f pose;
        OdomMsg(const rclcpp::Time& t, const Eigen::Vector2f& d) 
        : stamp(t), pose(d) {}
    };
    odom_node::odom_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<OdomMsg,boost::lockfree::capacity<20>> >quene ,std::string topic = "odom");
private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr& odom_data );
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_; //SharePtr便于管理
    std::shared_ptr<boost::lockfree::spsc_queue<OdomMsg,boost::lockfree::capacity<20>> > quene_;
    /*无锁队列 我们是单消费者单生产者情况,这样很快
    为了性能，不另外设置保存数据队列来处理历史数据，因为我们认为下位机是固定间隔上传数据的
    在抛出旧数据时我们只检查队列末尾的时间*/
};


class pose_extrapolator  //位姿推断器
{
public:

    pose_extrapolator(rclcpp::executors::MultiThreadedExecutor::SharedPtr exec,rclcpp::CallbackGroup::SharedPtr call_backs):
    exec_(exec),call_backs_(call_backs)
    {
    }


   void use_odom_node(std::string name = "odom_node", std::string topic = "odom")
    {
        odom_node_ = std::make_shared<odom_node>(name, odom_data_, topic);
        exec_->add_node(odom_node_);
    }
    
    void use_imu_node(std::string name = "imu_node", std::string topic = "imu")
    {
        imu_node_ = std::make_shared<Imu_node>(name, angle_velocityer_, topic);
        exec_->add_node(imu_node_);
    }
    
    void add_pose(Eigen::Vector2f &new_pose,float angle, rclcpp::Time& stamp);//进来一个新的最优位姿
    scout::Pose_anglef pose_extrapolator::extrapolator(const double& time_push);//外推
private:
    double time_push_;//上一次外推到哪了

    std::shared_ptr<Imu_node> imu_node_;
    std::shared_ptr<odom_node> odom_node_;
    const double odom_trust_time_;//里程计数据有效时间
    double odom_time_renew_;//里程计时间最新值
    Eigen::Vector2f velocity_;//线速度

    const double odom_trust_time_;//imu数据有效时间
    double imu_time_renew_;//imu时间最新值
    float imu_angular_velocity_;//imu传来的角速度
    std::shared_ptr<boost::lockfree::spsc_queue<odom_node::OdomMsg,boost::lockfree::capacity<50>>> odom_data_; //空间后面写成define好一点 必须保证消费比生产快好一点 雷达10hz
    std::shared_ptr<boost::lockfree::spsc_queue<Imu_node::ImuMsg,boost::lockfree::capacity<50>>> angle_velocityer_;// 里程计 imu传来的yaw轴角速度
    float linlinear_velocityer_from_odom_;//odom推断的线速度
    //float linlinear_velocityer_from_outside_;//下位机推导的线速度
    // std::queue<Eigen::Vector4f> angle_from_outside_;// 下位机推导的角度 四元数形式 节省计算
   // float imu_offset; //imu零飘

    float pose_queue_duration_;
    std::deque<scout::Pose_time> pose_data_;//位姿队列
    Eigen::Vector2f velocity_from_poses_;
    float angular_velocity_from_poses_;
    
    rclcpp::CallbackGroup::SharedPtr call_backs_;  //多线程回调函数组
    rclcpp::executors::MultiThreadedExecutor::SharedPtr exec_;  //多线程接口   
};

inline float QuaternionToAngle(const Eigen::Quaterniond& quaternion) 
{//将四元数变为z轴转动      防止eigen除0以及进行底层优化  再角度变化很小时,angle接近于0，会产生很大的浮点误差
    return  std::atan2(2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y())
    , 1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));
}

}//namespace
#endif