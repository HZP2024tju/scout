#include "imu.hpp"

namespace  sense
{
    //构造函数
Imu_node::Imu_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<ImuMsg,boost::lockfree::capacity<50>>>quene ,std::string topic )
        :Node(name),quene_(quene)
{
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(topic,10,
    [this](sensor_msgs::msg::Imu::SharedPtr imu_data){this->imu_callback(imu_data);} ); //传入的回调函数必须是一个实例
}

void Imu_node::imu_callback(const sensor_msgs::msg::Imu::SharedPtr& imu_data ) //ros2习惯写法
{//话题回调函数   基于标准消息
    quene_->push( ImuMsg(imu_data->header.stamp,imu_data->angular_velocity.z));//仅推入z轴数据
}

odom_node::odom_node(std::string name,std::shared_ptr<boost::lockfree::spsc_queue<OdomMsg,boost::lockfree::capacity<50>>>quene ,std::string topic )
        :Node(name),quene_(quene)
{
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(topic,10,
    [this](nav_msgs::msg::Odometry::SharedPtr odom_data){this->odom_callback(odom_data);} ); //传入的回调函数必须是一个实例
}

void odom_node::odom_callback(const nav_msgs::msg::Odometry::SharedPtr& odom_data ) //ros2习惯写法
{//话题回调函数   基于标准消息
    quene_->push( OdomMsg(odom_data->header.stamp,Eigen::Vector2f(odom_data->pose.pose.position.x,odom_data->pose.pose.position.y)) );//推入里程计推导的位姿数据
}

pose_extrapolator::pose_extrapolator(rclcpp::executors::SingleThreadedExecutor::SharedPtr exec, scout::Options* options):
exec_(exec),imu_trust_time_(options->imu_trust_time),odom_trust_time_(options->odom_trust_time),
pose_trust_time_(options->pose_trust_time)
{

}

void pose_extrapolator::add_pose(Eigen::Vector2f &new_pose,float& angle, rclcpp::Time& stamp)
{                                                                   //一般与laser共用一个stamp

    if (!pose_data_.empty() && stamp.seconds() <= pose_data_.back().stamp.seconds()) 
    {
        RCLCPP_ERROR(odom_node_->get_logger(), "时间戳未递增，忽略此位姿");
        return; 
    }

    pose_data_.emplace_back(stamp,new_pose,angle);
    double time_now = stamp.seconds();
    imu_time_push_ = time_now;
    odom_time_renew_ = time_now;
    
    if(pose_data_.size()<=1)
    {//大概是第一次开始
        return;
    }

    while (pose_data_.size() >2&&  ((stamp - pose_data_[0].stamp).seconds()) >= pose_trust_time_)
    {                                       //嵌入式平台算double很费时间
        pose_data_.pop_front();
    }

    double delta_time = (stamp- pose_data_.front().stamp).seconds(); 
    delta_time = delta_time > 1e-7 ? delta_time :1e-7;
    velocity_from_poses_= (new_pose -pose_data_.front().pose )/delta_time;
    angle_velocity_from_poses_ = (angle - pose_data_.front().yaw)/delta_time;

    while (angle_velocityer_->read_available()&& angle_velocityer_->front().stamp.seconds() < time_now)
    {//计算角度     这里频繁的访问这个无锁队列性能会被拉低吗?                               这里不需要减去trust 陀螺仪高频性能好
        angle_velocityer_->pop();
    }
    if(angle_velocityer_->read_available())
    {
        angle_velocity_from_imu_ = angle_velocityer_->front().angular_velocity;//耦合有点严重
        imu_time_newest_ = angle_velocityer_->front().stamp.seconds();//陀螺仪最新时间
    } 
    while (odom_data_->read_available() && odom_data_->front().stamp.seconds() < time_now-odom_trust_time_)
    {//计算速度
        odom_data_->pop();//弹出非法位姿
    }
}

//按时间进行姿态外推 不能倒着推
void pose_extrapolator::extrapolator(const double& time_push,Pose_anglef& pose_push)
{//time:需要推进到的时间
    double base_pose_time = pose_data_.back().stamp.seconds();//省一点时间
    if (base_pose_time > time_push || imu_time_push_ > time_push || odom_time_renew_ > time_push)
    {//这种情况应该不会发生吧
        RCLCPP_ERROR(odom_node_->get_logger(),"外推错误");
        return;
    }

    if(angle_velocityer_->read_available() || time_push< imu_trust_time_ + imu_time_newest_)
    {//imu倾向于逐个推进
        if(imu_time_newest_ > time_push)
        {//还不可以继续外推
            pose_push.yaw += std::abs(time_push - imu_time_push_)*angle_velocity_from_imu_ ;
        } 
        else
        {//可以继续推进 time_new < time_push
            while(angle_velocityer_->read_available()&&imu_time_newest_ < time_push)
            {
                pose_push.yaw += (imu_time_newest_ - imu_time_push_) * angle_velocity_from_imu_ ; //走完
                imu_time_push_ = imu_time_newest_;
                angle_velocityer_->pop();
                if(angle_velocityer_->read_available())
                {//如果还可以继续读
                    imu_time_newest_ = angle_velocityer_->front().stamp.seconds();
                    angle_velocity_from_imu_ = angle_velocityer_->front().angular_velocity;//更新角速度
                }
                else
                {
                    break;
                }
                
            }
            pose_push.yaw += (time_push - imu_time_push_) * angle_velocity_from_imu_ ;//走剩下的
        }
    
        imu_time_push_ = time_push;//有点屎
    }
    else
    {
        pose_push.yaw += angle_velocity_from_poses_ * std::abs(time_push - base_pose_time);//这里缺少数据直接用均匀角速度没问题
    }

    if (time_push < odom_time_renew_+odom_trust_time_)
    {//如果可直接外推   
        pose_push.pose +=  velocity_from_odom_*(time_push - base_pose_time);
        return;
    }
    else if(odom_data_->read_available() >= 2)
    {//只要还有数据就证明odom仍在正常工作,偏向相信odom的数据  我们倾向于一帧只外推一次

        odom_node::OdomMsg odom_old = odom_data_->front();
        while (odom_data_->read_available() != 1)
        {//计算速度 一口气推到底
            odom_data_->pop();
        }
        odom_time_renew_=odom_data_->front().stamp.seconds();//更新推进时间
        double dt = odom_time_renew_ - odom_old.stamp.seconds();
        velocity_from_odom_ = (odom_data_->front().pose - odom_old.pose) / (dt> 1e-7? dt:1e-7);//防止除0

        pose_push.pose +=  velocity_from_odom_*(time_push - base_pose_time);
    }
    else
    {
        pose_push.pose +=  velocity_from_poses_ *(time_push - base_pose_time);
    }
}


}//namespace