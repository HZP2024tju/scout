#ifndef _FILTER_HPP_
#define _FILTER_HPP_
#include "Eigen/Dense"
#include "thread.hpp"
#include "absl/container/flat_hash_map.h"//absl 20220623
#include <omp.h>
#include <vector>
#include <unordered_set>
#include <memory>
#include <random>

#include "base_struct.hpp"
namespace sense
{
Eigen::Vector2f volex_filter(const std::vector<Eigen::Vector2f>& point_cloud,Eigen::AlignedBox2i cells_box,float resolution);

}

#endif