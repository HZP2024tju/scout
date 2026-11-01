#include "filter.hpp"
#include "Eigen/Dense"
#include "thread.hpp"

#include <omp.h>
#include <vector>
#include <unordered_set>
#include <memory>
#include <random>
namespace sensor
{
//返回滤波后的点
std::unique_ptr<std::vector<Eigen::Vector2f>> volex_filter(const std::vector<Eigen::Vector2f>& point_cloud,Voxoptions& options)
{
  std::minstd_rand0 generator;
  auto result=std::make_unique<std::vector<Eigen::Vector2f>>();//可用的点   这样更快吧
  result->reserve(point_cloud.size());

  std::unordered_map<uint64_t,std::pair<int, int>> map;//哈希表
  map.reserve(point_cloud.size());//@TODO 对于2d雷达或许这种方法最有效 待测 与absl哈希表比较
  //std::vector<int>use_point
  int gx,gy;
  uint64_t key;

  //这里不太能多线程
  for (int i = 0; i < point_cloud.size(); i++) 
  {//遍历点云   这样写开销小 不进行拷贝
    const auto& point = point_cloud[i]; //零拷贝
    gx= std::lround( point.x()/options.vox_lenth ) ; //carto是这么写的 这样真的比floor好吗?
    gy= std::lround( point.y()/options.vox_lenth ) ;//注意体素长是现实长度,应该比分辨率稍小
    key = (uint64_t(uint32_t(gx)) << 32) | uint32_t(gy);
  
    auto& hash = map[key];//值引用
    if (hash.first==0) //我记得好像find不到会自动申请,所以是否auto要放里面?
    {//没重复 插入成功
      
      hash.first=1;
      hash.second=i;
    }
    else
    {//如果重复了 插入失败
      hash.first++;
      std::uniform_int_distribution<> distribution(1,hash.first);
      if (distribution(generator) == hash.first) 
      {//概率技巧 随着增长每个点有同等概率替换旧点
        hash.second=i;
      }
    }
  }

  for (const auto& use_point : map)
  {
    (*result).emplace_back(point_cloud[use_point.second.second]);
  }
  
  return result;
}

} // namespace sensor
