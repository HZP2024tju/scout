#include <vector>
#include "submap.hpp"


scout::Submap2D::Submap2D(const Eigen::Vector2f& world_origin, std::unique_ptr<grid2d> grid,
           const std::vector<float> * table)
           :origin(world_origin),uint15tofloat_table_(table)
{
    grid_ = std::move(grid);//这里应该是视为冻结时的子图
};