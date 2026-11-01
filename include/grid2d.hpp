#ifndef _GRID2D_HPP_
#define _GRID2D_HPP_
#include <vector>
#include <algorithm>
#
namespace scout
{
class grid2d
{
public:
    grid2d(/* args */);
    ~grid2d();
private:
 // MapLimits limits_;
  std::vector<uint16_t> probability_cells_;//本体
  float min_probability_;
  float max_probability_;//概率上下界
  std::vector<int> update_indices_;//本次更新的栅格 已扁平化   为了在高速搜索时加快效率

  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;//AABB盒
  const std::vector<float>* uint15toflaot_table_;
};

 

}
#endif