#ifndef _POSE_GRAPH_HPP_
#define _POSE_GRAPH_HPP_
#include "base_struct.hpp"
namespace graph
{
struct constraint 
{//约束结构体
scout::Pose_anglef pose;//节点局部位姿
uint32_t submap_id;  // 'i' in the paper.
uint32_t node_id;      // 'j' in the paper.
enum Tag { FRONT_SUBMAP, BACKWARD_SUBMAP } tag;
};

struct submap_back
{//后端子图结构体
   //@TODO 这怎么摆才快一点
scout::Pose_anglef origin;//子图的全局位姿
std::shared_ptr<scout::grid2d> grid;
uint32_t id;
};

struct node_scan
{

    std::shared_ptr<std::vector<Eigen::Vector2f>> point_cloud;//点云数据
};
class optimit_graph
{//后端子图类
    public:
    private:
    

};
}//namespace
#endif