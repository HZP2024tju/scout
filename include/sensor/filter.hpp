#ifndef _FILTER_HPP_
#define _FILTER_HPP_
#include "Eigen/Dense"
#include "absl/container/flat_hash_map.h"//absl 20220623
#include <omp.h>
#include <vector>
#include <unordered_set>
#include <memory>
#include <random>

#include "base_struct.hpp"
namespace sense
{
class filters
{
    public:
    filters(scout::Options* option);
    void volex_filter(std::unique_ptr<std::vector<Eigen::Vector2f>>& point_cloud);
    private:
    float vox_lenth_;
};

}

#endif