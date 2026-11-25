#include <vector>
#include "submap.hpp"
#include "compute.hpp"
namespace scout{
scout::Submap2D::Submap2D(const Pose_anglef& world_origin,const std::shared_ptr<std::vector<float>> table,Options* option)
           :origin(world_origin),uint15tofloat_table_(table),resolution_(option->resolution),
           grid_(option),
    cos_yaw_(std::cos(world_origin.yaw)),
    sin_yaw_(std::sin(world_origin.yaw)),
    inv_resolution_(1.0f / option->resolution)
    {};

ActiveSubmaps2D::ActiveSubmaps2D(Options* option):insert_max_(option->insert_max_)
{
    insert_count_=0;
    map_num=0;
    macthing_map_=0;
    building_map_=0;
}

//按全局坐标获取概率
inline uint16_t ActiveSubmaps2D::GetProbability_global(float x,float y)
{
    return  submaps_[macthing_map_].GetProbability_global(x,y);
}

inline uint16_t ActiveSubmaps2D::GetProbability_local(float x,float y)
{
    return  submaps_[macthing_map_].GetProbability_local(x,y);
}


//用世界位姿来拿取概率值 先转到子图的局部位姿再计算概率
inline uint16_t Submap2D::GetProbability_global(float x,float y)
{
    return grid_.Get_Probability(
    compute::fast_floor(( (x-origin.pose.x())* cos_yaw_+(y-origin.pose.y())*sin_yaw_)*inv_resolution_),
    compute::fast_floor((  (y-origin.pose.y())*cos_yaw_- (x-origin.pose.x())*sin_yaw_)*inv_resolution_));
}

inline uint16_t Submap2D::GetProbability_local(float x,float y)
{
    return grid_.Get_Probability(x*inv_resolution_,y*inv_resolution_);
}

void ActiveSubmaps2D::insert(Pose_anglef& pose_best_global,std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_ptr)
{
    
}
}//space