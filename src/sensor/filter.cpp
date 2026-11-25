#include "filter.hpp"
#include "settings.hpp"
#include "compute.hpp"

namespace sense
{
//返回滤波后的点
filters::filters(scout::Options* option):vox_lenth_(option->vox_lenth)
{}

//体素滤波，同时过滤掉了无效点
void filters::volex_filter(std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud)
{
  std::minstd_rand0 generator;

  absl::flat_hash_map<uint64_t, std::pair<int, int>> map;//哈希表 
  map.reserve(point_cloud->size());//@TODO 对于2d雷达或许这种方法最有效 待测 
  //std::vector<int>use_point
  int gx,gy;
  uint64_t key;
  auto& vec = *point_cloud; // 拿到真实的 vector 引用

  //这里不太能多线程
  for (size_t i = 0; i < point_cloud->size(); i++) 
  {//遍历点云   这样写开销小 不进行拷贝
    const auto& point = vec[i]; //零拷贝
    if(point == Point_Unknow)
    {
      continue;
    }

    gx= compute::fast_floor( point.x()/vox_lenth_ ) ; //carto是这么写的 这样真的比floor好吗?
    gy= compute::fast_floor( point.y()/vox_lenth_ ) ;//注意体素长是现实长度,应该比分辨率稍小
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

  std::vector<bool> points_used(vec.size(), false);
  
  for (const auto& use_point : map)
  {
      points_used[use_point.second.second] = true;   
  }

  uint32_t j=0;
  for (int i = 0; i< vec.size();++i)
  {
    if (points_used[i] == true)
    {
      vec[j] = vec[i];
      ++j;
    }    
  }
  vec.resize(j);
}

void kalman_filter()
{//@todo 里程计+imu 联合紧耦合滤波计算yaw会更好吗？

}
} // namespace sense
