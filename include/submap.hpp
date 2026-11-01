#ifndef _SUBMAP_HPP_
#define _SUBMAP_HPP_
#include <memory>
#include <Eigen/Dense>
#include "grid2d.hpp"
namespace scout
{
class Submap2D 
{
    //子图
 public:
    Submap2D(const Eigen::Vector2f& world_origin, std::unique_ptr<grid2d> grid,
           const std::vector<float> * table);

 private:
    Eigen::Vector2f origin;//子图全局位姿
    std::unique_ptr<grid2d> grid_;//子图对应的概率栅格
    std::unique_ptr<const std::vector<float>> uint15tofloat_table_;//uint16/2转float的表格
};

//活跃子图
class ActiveSubmaps2D {
 public:
 
 private:
    //std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
    std::unique_ptr<grid2d> CreateGrid(const Eigen::Vector2f& origin);

    std::vector<std::shared_ptr<Submap2D>> submaps_;
    //std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
    const std::vector<float> conversion_tables_;//uint16/2转float的表格
};

}
#endif