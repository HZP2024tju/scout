#include "imu.hpp"

namespace  sense
{
    //构造函数
Imu_node::Imu_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<ImuMsg,boost::lockfree::capacity<20>>>quene ,std::string topic )
        :Node(name),quene_(quene)
{
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(topic,10,
    [this](sensor_msgs::msg::Imu::SharedPtr imu_data){this->imu_callback(imu_data);} ); //传入的回调函数必须是一个实例
}

void Imu_node::imu_callback(const sensor_msgs::msg::Imu::SharedPtr& imu_data ) //ros2习惯写法
{//话题回调函数   基于标准消息
    quene_->push( ImuMsg(imu_data->header.stamp,imu_data->angular_velocity.z));//仅推入z轴数据
}

odom_node::odom_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<OdomMsg,boost::lockfree::capacity<20>>>quene ,std::string topic )
        :Node(name),quene_(quene)
{
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(topic,10,
    [this](nav_msgs::msg::Odometry::SharedPtr odom_data){this->odom_callback(odom_data);} ); //传入的回调函数必须是一个实例
}

void odom_node::odom_callback(const nav_msgs::msg::Odometry::SharedPtr& odom_data ) //ros2习惯写法
{//话题回调函数   基于标准消息
    quene_->push( OdomMsg(odom_data->header.stamp,Eigen::Vector2f(odom_data->pose.pose.position.x,odom_data->pose.pose.position.y)) );//推入里程计推导的位姿数据
}


void pose_extrapolator::add_pose(Eigen::Vector2f &new_pose,float angle, rclcpp::Time& stamp)
{                                                                   //一般与laser共用一个stamp
    pose_data_.emplace_back(stamp,new_pose,angle);
    while (pose_data_.size() >2&& (static_cast<float>((stamp - pose_data_[1].stamp).seconds())) >= pose_queue_duration_)
    {                                       //嵌入式平台算double很费时间
        pose_data_.pop_front();
    }
    float delta_time = (stamp- pose_data_.front().stamp).seconds(); 
    velocity_from_poses_= (new_pose -pose_data_.front().pose )/delta_time;
    angular_velocity_from_poses_ = (angle - pose_data_.front().yaw)/delta_time;
}

//按时间进行姿态外推
scout::Pose_anglef pose_extrapolator::extrapolator(const double& time_push)
{//time:需要推进到的时间
    scout::Pose_time Pose_new = pose_data_.back();//直接先拿取最新位姿
    double base_pose_time = Pose_new.stamp.seconds();//省一点时间
    if (base_pose_time > time_push)
    {//这种情况应该不会发生吧
        RCLCPP_ERROR(odom_node_->get_logger(),"外推错误");
        return;
    }
    
    while (angle_velocityer_->read_available()&& angle_velocityer_->front().stamp.seconds() < time_push)
    {//计算角度     这里频繁的访问这个无锁队列性能会被拉低吗?
        time_new = angle_velocityer_->front().stamp.seconds();
        if (time_new < time_origin)
        {
            angle_velocityer_->pop();
            continue;
        }
        else
        {
            Pose_new.yaw += angle_velocityer_->front().angular_velocity * (time_new-time_origin);//推进   @todo 卡尔曼滤波待测试
            time_origin = time_new;
            angle_velocityer_->pop();   
        }
        imu_angular_velocity_ = angle_velocityer_->front().angular_velocity;
    }

    
    if (time_push<odom_time_renew_)
    {//如果可直接外推   
        Pose_new.pose += velocity_*(time_push - base_pose_time);
        return scout::Pose_anglef{Pose_new.pose,Pose_new.yaw};//这里会因为构造而损失性能吗 
    }
    else if(time_push - odom_time_renew_ < odom_trust_time_ || odom_data_->read_available())
    {//只要还有数据就证明odom仍在正常工作,偏向相信odom的数据   
        Eigen::Vector2f odom_old;
        Eigen::Vector2f odom_new;
        while (odom_data_->read_available())
        {//计算速度 一口气推到底
            if (odom_data_->front().stamp.seconds() < base_pose_time)
            {
                odom_data_->pop();//弹出非法位姿
                continue;
            }
        }
        odom_old = 
        while (odom_data_->read_available())
        {//计算速度 一口气推到底
            if (time_new < Pose_new.stamp.seconds())
            {
                odom_data_->pop();//弹出非法位姿
                continue;
            }
            odom_time_renew_ = time_new; //记录最新推进的odom位姿
            //如果可以更新速度，那么就更新
            velocity_ = (odom_data_->front().pose - Pose_new.pose)/(time_push-time_origin);//需要保证雷达扫一圈比里程计更慢 也就是说雷达10hz 里程计至少30hz
        }

        Pose_new.pose +=  velocity_*(time_push - time_origin);
    }
    
    
    Pose_new.pose +=  velocity_from_poses_ *(time_push - base_pose_time);
    return scout::Pose_anglef{Pose_new.pose,Pose_new.yaw};//这里会因为构造而损失性能吗 
}


}//namespace