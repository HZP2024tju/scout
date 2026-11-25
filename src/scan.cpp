#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "Eigen/Dense"
#include <omp.h>

#include "sensor/filter.hpp"
#include "scan.hpp"
#include "compute.hpp"
namespace scout
{//建图前端
scan::scan(rclcpp::executors::SingleThreadedExecutor::SharedPtr exec,Options *option):exec_(exec),options_(option),
 laser_offset_x(option->laser_offset_x),laser_offset_y(option->laser_offset_y),laser_offset_angle(option->laser_offset_angle),
 point_cloud_size(option->point_cloud_size),use_particle_scan(option->use_particle_scan),ceres_max_step(option->ceres_max_step),
 ceres_max_time(option->ceres_max_time),angle_cost_weight_(option->angle_cost_weight_),translate_cost_weight_(option->Translate_weight),
 active_submaps_(option),Front_ceres_weight(Front_ceres_weight)
{
    extrapolator_.use_imu_node();
    extrapolator_.use_odom_node();//@TODO这里要根据options设置
}

Front_Cost_Function2D::Front_Cost_Function2D(const Submap2D* submap,
                        const std::unique_ptr<std::vector<Eigen::Vector2f>>& global_point_cloud_ptr,const Pose_anglef& pose_origin,double weight)
      : point_cloud_(global_point_cloud_ptr),
        submap_ptr_(submap),pose_origin_(pose_origin) ,weight_(weight)
        {};//这里我们选择使用子图传过来grid

std::unique_ptr<std::vector<Eigen::Vector2f>> scan::fix_pointcloud_with_imu(float* range_arr ,size_t& point_count, Pose_anglef& pose_push,double time_base)
{
    float angle_step;
    float range;
    auto point_cloud = std::unique_ptr<std::vector<Eigen::Vector2f>>();
    point_cloud->reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) 
    {
        time_base += time_increment;//@TODO 有于基准是上一次的位姿 我们认为第一条射线是隔了一个时间段
        extrapolator_.extrapolator(time_base,pose_push);//然后这里是外推出最新的位姿了
        
        range=range_arr[i];
        if(std::isfinite(range) && range>range_min&&range<range_max)
        {                  
            angle_step = pose_push.yaw + (laser_offset_angle +laser_start + laser_follow_clock*i*angle_increment -angle_min);//我们默认从0°开始扫描
            point_cloud->emplace_back(Eigen::Vector2f{pose_push.pose.x()+range*cos(angle_step),pose_push.pose.y()+range*sin(angle_step)});//全局坐标系下的点云
        }
        else
        {
            point_cloud->emplace_back(Eigen::Vector2f{NAN, NAN}); //无效 后面一定要加以判断！
        }
    }
    return point_cloud;
}

//用实际位姿计算点云得分 直接查看原始栅格  
float scan::compute_pointcloud_score(std::vector<Eigen::Vector2f>&point_cloud) 
{              
    float candidate_score = 0.f;
    uint16_t probability;
    //@TODO这里需要测试并行开销
    for (const Eigen::Vector2f& xy_index : point_cloud)
    {
        probability = active_submaps_.GetProbability_global(xy_index.x() ,xy_index.y());//这里是全局位姿
        if (probability!=unknow_value)
        {
            candidate_score += probability;//unknow_value是0
        }
    }
    candidate_score /= static_cast<float>(point_cloud.size());
    return candidate_score;
}

//依赖当前推断位姿和点云用窗口粗匹配 返回最优位姿
void scan::scan_particle_match(Pose_anglef& pose_origin, std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_ptr)
{//@TODO再review一遍
  //首先按步长生成搜索空间
    auto& point_cloud = *point_cloud_ptr; //避免深拷贝
    float max_scan_range = 9.f *options_->resolution* options_->resolution;//防止点云为空 3个格子距离的平方

    float range ;
    for (const Eigen::Vector2f& point : point_cloud)
    {
        range = (pose_origin.pose.x()-point.x())*(pose_origin.pose.x()-point.x())+(pose_origin.pose.y()-point.y())*(pose_origin.pose.y()-point.y()); //
        max_scan_range = std::max(range, max_scan_range);
    }
    max_scan_range = std::sqrt(max_scan_range);

    const double kSafetyMargin = 1. - 1e-3;//为了防止浮点误差漏掉格子 乘一个数  细节
    const double search_step_angle =  kSafetyMargin * 
                std::acos(1. - (resolution_*resolution_) /
                (2. * max_scan_range*max_scan_range)) ;//余弦定理 每个搜索变化的角度

    const uint32_t num_search_angle = std::ceil(angle_search_window / search_step_angle); //这里一共有几个搜索角? 
    const uint32_t num_search_linear =std::ceil(linear_search_window /  search_step_linear);//有几个线性搜索点？

    const uint32_t point_count = point_cloud.size();

    std::vector<ThreadBestResult>  score_result; //各个线程得到的最好分数与位姿 对齐64Byte的缓存行 在其它机器上可能就要改一下@TODO
    score_result.resize(SCOUT_OMP_THREAD_NUM_FRONT);
    //圆形或者扇形搜索空间内进行点云变换并打分  @todo 这里需要测试并行是否比串行快
    #pragma omp parallel num_threads(SCOUT_OMP_THREAD_NUM_FRONT) 
    {
        score_result[omp_get_thread_num()].score =0;
        float score;
        float delta_yaw;//相对于原本的点云旋转了多少角度
        float cos_d;
        float sin_d;
        float point_dx;
        float point_dy;
        float linear_dx;
        float linear_dy;
        float current_pose_x;
        float current_pose_y;
        uint16_t probability_temp;
        #pragma omp for schedule(static)
        for (size_t i = 0; i < num_search_angle; i++) //交给两个线程执行这个for
        {
            current_pose_x=pose_origin.pose.x();
            current_pose_y=pose_origin.pose.y();
            delta_yaw = angle_search_window/2 - i*search_step_angle;//相对于原本的点云旋转了多少角度
            cos_d = std::cos(delta_yaw);
            sin_d = std::sin(delta_yaw);
            linear_dx = std::cos(delta_yaw+pose_origin.yaw)*search_step_linear;
            linear_dy = std::sin(delta_yaw+pose_origin.yaw)*search_step_linear;
            for (size_t j = 0; j < num_search_linear; j++)
            {//按角度线性搜索
                score=0;
                for (size_t k = 0; k < point_count; k++)
                {//对点云进行变换
                    point_dx = point_cloud[k].x() - pose_origin.pose.x();
                    point_dy = point_cloud[k].y() - pose_origin.pose.y(); //这里如果提出来变成数组的话，下面就可以SIMD化@TODO
                    // 旋转后
                    probability_temp = active_submaps_.GetProbability_global(current_pose_x+ point_dx * cos_d - point_dy * sin_d,
                                                                current_pose_y + point_dx * sin_d + point_dy * cos_d);//这里直接世界位姿就行，由世界位姿转到子图坐标系再计算匹配
                    if (probability_temp!=unknow_value) //@TODO这里换成更易读的eigen形式
                    {
                        score += probability_temp;//unknow_value是0
                    }
                }
                
                score *= std::exp(-std::pow((std::hypot(current_pose_x-pose_origin.pose.x(), current_pose_y-pose_origin.pose.y()) * translate_cost_weight_+
                            std::abs(delta_yaw) * angle_cost_weight_),2))/ static_cast<float>(point_cloud.size());;//乘一个惩罚系数，距离原始位姿越远越需要惩罚@TODO hypot和pow运行较慢待测试
                               
                if (score > score_result[omp_get_thread_num()].score)
                {//确定最佳位姿
                    score_result[omp_get_thread_num()].pose = Pose_anglef{current_pose_x,current_pose_y,pose_origin.yaw+delta_yaw};
                    score_result[omp_get_thread_num()].score =  score;//这里不需要设计best_score 本来数组里存的就是
                }
                
                current_pose_x += linear_dx;
                current_pose_y += linear_dy;
            }
        }
    } //omp

    uint16_t index_best_thread=0;
    float best_score=0;
    for (size_t i = 0; i < score_result.size(); ++i)
    {
        if (score_result[i].score > best_score)
        {
            best_score = score_result[i].score ;
            index_best_thread = i;
        }
    }
    
    //下面计算旋转的点云
    float delta_yaw = score_result[index_best_thread].pose.yaw - pose_origin.yaw;//相对于原本的点云旋转了多少角度
    float cos_d = std::cos(delta_yaw);
    float sin_d = std::sin(delta_yaw);
    float point_dx;
    float point_dy;
    
    for (size_t k = 0; k < point_count; ++k)
    {//对点云进行变换
        point_dx = point_cloud[k].x() - pose_origin.pose.x();
        point_dy = point_cloud[k].y() - pose_origin.pose.y(); 
        // 旋转后
        point_cloud[k] =  Eigen::Vector2f{score_result[index_best_thread].pose.pose.x() + point_dx * cos_d - point_dy * sin_d,
                                          score_result[index_best_thread].pose.pose.y() + point_dx * sin_d + point_dy * cos_d};
    }
    pose_origin = score_result[index_best_thread].pose;
}//void

void scan::scan_ceres_match(Pose_anglef& pose_origin,std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_ptr)//ceres求解最优位姿
{
    ceres::Problem problem;
    double pose_ceres[3] = {pose_origin.pose.x(),pose_origin.pose.y(),pose_origin.yaw};
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Front_Cost_Function2D,ceres::DYNAMIC ,3>(
new Front_Cost_Function2D(active_submaps_.GetActiveSubmapPtr(), point_cloud_ptr, pose_origin,Front_ceres_weight/static_cast<double>(point_cloud_ptr->size()))
      ,point_cloud_ptr->size())
      ,nullptr,pose_ceres );
    
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor2D,1 ,3>(
      new RotationDeltaCostFunctor2D(pose_origin.yaw,angle_cost_weight_) ),nullptr,pose_ceres);

    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,2 ,3>(
      new TranslationDeltaCostFunctor2D(pose_origin.pose.x(),pose_origin.pose.y() ,translate_cost_weight_) ),nullptr,pose_ceres);

    ceres::Solver::Options option_ceres;
    option_ceres.linear_solver_type = ceres::DENSE_QR;   // 3 个变量，dense 最快
    option_ceres.max_num_iterations = ceres_max_step;
    option_ceres.max_solver_time_in_seconds = ceres_max_time;      // 单帧最多 50ms，超了就用当前结果
    option_ceres.num_threads = 1;                        // @TODO 确定此处的线程 小问题多线程反而有开销
    option_ceres.minimizer_progress_to_stdout = false;   // 别打印


    ceres::Solver::Summary summary;
    ceres::Solve(option_ceres, &problem, &summary);//@TODO这里需要测试运行时间

    if (!summary.IsSolutionUsable()) 
    {
        return;   // 保持 pose_origin 不变，用初值（里程计）
    }
    Eigen::Vector2f pose_best=Eigen::Vector2f{pose_ceres[0],pose_ceres[1]};
    Eigen::Rotation2D rotation(pose_ceres[2] - pose_origin.yaw);
    for (uint32_t i = 0; i < point_cloud_ptr->size(); ++i)
    {//转为最优位姿下的全局点云
        (*point_cloud_ptr)[i] = rotation*((*point_cloud_ptr)[i] - pose_origin.pose) + pose_best;//@TODO 来点xsimd
    }
    
    pose_origin.pose.x() = static_cast<float>(pose_ceres[0]);
    pose_origin.pose.y() = static_cast<float>(pose_ceres[1]);
    pose_origin.yaw      = static_cast<float>(pose_ceres[2]);
}


void scan::scan_build_front()
{
    if (!point_cloud_data_->read_available())
    {//没数据就返回
        return;
    }
    Pose_time pose_best_;
    if (!scan_first)
    {//如果是第一次
        pose_best_.pose = Eigen::Vector2f(0,0);
        pose_best_.yaw = 0;
        pose_best_.stamp = laser_node_->now();//好神秘的获取时间方式
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
        //这里要插入动态滤波器
        pose_best_ = extrapolator_.pose_data_.back();
    }
    
    sensor_msgs::msg::LaserScan::SharedPtr current_scan;

    float* range_arr;
    size_t point_count;
    double time_push_temp;
    scout::Pose_anglef pose_push_temp;

    while (point_cloud_data_->pop(current_scan)) 
    {//取出数据 直到没有为止
        time_increment=current_scan->time_increment;//看看本次点间隔多久
        angle_increment=current_scan->angle_increment;
        time_push_temp=pose_best_.stamp.seconds();
        pose_push_temp.pose=pose_best_.pose;            //推导起始
        pose_push_temp.yaw=pose_best_.yaw;    
        
        range_arr = current_scan->ranges.data();
        point_count = current_scan->ranges.size();//数据

        std::unique_ptr<std::vector<Eigen::Vector2f>> point_cloud_temp;
        //下面进行去畸变
        if (use_fix_pointcloud_with_imu)
        {
            point_cloud_temp= fix_pointcloud_with_imu(range_arr,point_count,pose_push_temp,time_push_temp);//去畸变的时候顺手转成局部的
        }
        else
        {//直接将最后一个点作为此时的位姿 一般是测试的时候才会这么做
            time_push_temp += time_increment*point_count;
            extrapolator_.extrapolator(time_push_temp,pose_push_temp);//然后这里是外推出最新的位姿了

            point_cloud_temp->reserve(point_count);
            float range;
            float angle_step;//计算每条射线的角度
            for (size_t i = 0; i < point_count; ++i) 
            {  //转换为全局点云
                range=range_arr[i];
                if(std::isfinite(range) && range>range_min&&range<range_max)
                {                  
                    angle_step = pose_push_temp.yaw + (laser_offset_angle +laser_start + laser_follow_clock*i*angle_increment -angle_min);//我们默认从0°开始扫描
                    point_cloud_temp->emplace_back(Eigen::Vector2f{pose_push_temp.pose.x()+range*cos(angle_step),pose_push_temp.pose.y()+range*sin(angle_step)});
                }
                else
                {
                    point_cloud_temp->emplace_back(Point_Unknow); //无效 后面一定要加以判断！
                }
            }
        }

        filter_.volex_filter(point_cloud_temp);//体素滤波 这里没问题吧  同时过滤掉无效点

        if (use_particle_scan)
        {
            scan_particle_match(pose_push_temp,point_cloud_temp);//将pose_push_temp变为粗扫后的位姿
        }    

        if (active_submaps_.map_num==0)
        {//如果刚开始还没有活跃子图
            /*直接向子图插入点云并 建立约束 检查扩张*/
        }
        else
        {//已经有子图了 那就匹配
            scan_ceres_match(pose_push_temp,point_cloud_temp);//ceres求解最优位姿
        }   
    }//while
}


}//namespace