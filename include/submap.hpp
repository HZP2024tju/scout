#ifndef _SUBMAP_HPP_
#define _SUBMAP_HPP_
#include <memory>
#include <Eigen/Dense>
#include "grid2d.hpp"
#include "ceres/jet.h"

#include "compute.hpp"
#include "base_struct.hpp"
namespace scout
{
// 辅助：取 Jet/double 的数值部分
template <typename S>
inline double scalar_part(const S& v) { return static_cast<double>(v); }
template <typename S, int N>
inline float scalar_part(const ceres::Jet<S, N>& v) { return static_cast<double>(v.a); }

class Submap2D
{
    //子图
 public:
   scout::Submap2D::Submap2D(const Pose_anglef& world_origin,const std::shared_ptr<std::vector<float>> table,Options* option);
   //起始世界位姿       转换表    设置   

   inline uint16_t GetProbability_global(float x,float y);

    inline uint16_t GetProbability_local(float x,float y);

   
template <typename T> //用local点云获取数据
inline T GetlocalLinearFloatProbability(T x,T y)//使用双线性插值获取可微概率 ceres用
const {//@TODO试试双三次插值看看
    //输入点的局部位姿
    int x0 = static_cast<int>(compute::fast_floor(scalar_part(x)*inv_resolution_ ));
    int y0 = static_cast<int>(compute::fast_floor(scalar_part(y)*inv_resolution_ ));//变为栅格点
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    
    if (!grid_.known_cells_box_.contains(Eigen::Vector2i(x0, y0)) ||!grid_.known_cells_box_.contains(Eigen::Vector2i(x1, y1)))
    {
      return T(unknow_value);
    }
    
    T dx = x * T(inv_resolution_)- T(x0);
    T dy = y * T(inv_resolution_)- T(y0);

    T p00 = T((*uint15tofloat_table_)[grid_.Get_ProbabilityUncheck(x0, y0)]); //前面已经检查过knowbox的话这里就不再检查溢出了
    T p10 = T((*uint15tofloat_table_)[grid_.Get_ProbabilityUncheck(x1, y0)]);
    T p01 = T((*uint15tofloat_table_)[grid_.Get_ProbabilityUncheck(x0, y1)]);
    T p11 = T((*uint15tofloat_table_)[grid_.Get_ProbabilityUncheck(x1, y1)]);

    T p0 = p00 * (T(1.0) - dx) + p10 * dx;
    T p1 = p01 * (T(1.0) - dx) + p11 * dx;
    
    return p0 * (T(1.0) - dy) + p1 * dy;
}

 private:
   uint32_t id;
   Pose_anglef origin;//子图全局位姿
   grid2d grid_;//子图对应的概率栅格
   float resolution_;
   int max_limit_;
   int min_limit_;
   float resolution_;
   // 初始化时算好，后续永不改变
   const float cos_yaw_;
   const float sin_yaw_ ;
   const float inv_resolution_;
   const std::shared_ptr<std::vector<float>> uint15tofloat_table_;//uint16/2转float的表格
};

//活跃子图管理
class ActiveSubmaps2D 
{
 public:
   ActiveSubmaps2D(Options* option);

  void insert(Pose_anglef& pose_best_global,std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_ptr);
  inline uint16_t GetProbability_global(float x,float y);
  inline uint16_t GetProbability_local(float x,float y);

  inline Submap2D* GetActiveSubmapPtr()
  {
    return &submaps_[macthing_map_];
  }

  void insert(Pose_anglef pose_best, std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud_ptr);//有点云，位姿就能自动选择需要的子图插入
  std::vector<Submap2D> submaps_; //两个map
  uint16_t scan_limit;
  uint8_t map_num;//当前活跃子图个数
  
private:
  uint8_t macthing_map_;//正在使用哪一张活跃地图作为匹配
  uint8_t building_map_;//正在构建那一张地图
  uint16_t insert_count_;//已经插入了多少帧扫描
  const uint16_t insert_max_;
};

}
#endif