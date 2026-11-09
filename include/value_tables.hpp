#ifndef _SCOUT_VALUE_TABLES_HPP_
#define _SCOUT_VALUE_TABLES_HPP_
#include <memory>
#include "compute.hpp"
#include <vector>
namespace scout{
std::unique_ptr<std::vector<float>> make_odd_table();
std::unique_ptr<std::vector<float>> make_Uint15tofloat_table();
}
#endif