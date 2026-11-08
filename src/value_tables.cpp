#include "value_tables.hpp"
#include "compute.hpp"
#include <vector>


namespace scout
{
    //做一个uint15odd更新的表格
std::unique_ptr<std::vector<float>> make_odd_table(float change,std::vector<float>& uint15tofloat_table)
    {
       // std::vecotr<>
       //std::
        auto result = std::make_unique<std::vector<float>>();
        result->reserve(32768);//[1,32767]到float的表 表示相应的概率经过hit/miss后的变化
        result->emplace_back(0);
        for (int i = 1; i <= 32767; i++)
        {
            result->emplace_back(compute::BoundFloat2Uint15
                (compute::Odds2P
                (compute::Odds(uint15tofloat_table[i])*(change/(1.f-change))))) ; //用旧odds乘变化值的odds就是新odds
                //再将 odds转为概率 变为uint15再存入表格 这样就做好了速算的准备
        }
        return result;
    }

std::unique_ptr<std::vector<float>> make_Uint15tofloat_table()
    {
        auto result = std::make_unique<std::vector<float>>();
        result->reserve(32768);//[1,32767]到float的表
        result->emplace_back(0);
        for (int i = 1; i <= 32767; i++)
        {
            result->emplace_back(compute::BoundUint15tofloat(i));
        }
        return result;
    }
}//namespace
