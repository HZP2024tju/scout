#ifndef _GRID2D_HPP_
#define _GRID2D_HPP_
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <memory>

#include "settings.hpp"
#include "base_struct.hpp"
namespace scout
{

//我们认为栅格的点向左下角对齐
class grid2d
{
public:
    grid2d(Options*option);//这里确定子图的栅格的大小
    ~grid2d();

    int Get_index(int x,int y);

    inline uint16_t Get_Probability(int x,int y);
    inline uint16_t Get_ProbabilityUncheck(int x,int y);

    void extend(Eigen::Vector2i& min,Eigen::Vector2i& max);
private:
 // MapLimits limits_;
    std::unique_ptr<std::vector<uint16_t>> probability_cells_;//本体
    std::vector<int> update_indices_;//本次更新的栅格 已扁平化   为了在高速搜索时加快效率
    uint32_t lenth_;
    uint32_t wide_;
    Eigen::Vector2i max_;//地图的最大囊括范围
    Eigen::Vector2i min_;//地图的最小囊括范围  同时也是获取index的索引
    
    float min_probability_;
    float max_probability_;//概率上下界
    
    Eigen::AlignedBox2i known_cells_box_;//已探索边界
    Eigen::AlignedBox2i max_cells_box_;//最大边界
    const std::vector<float>* uint15toflaot_table_;
};
}
#endif