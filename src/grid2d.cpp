#include "grid2d.hpp"
#include "compute.hpp"
namespace scout
{

grid2d::grid2d(Options*option):
max_(compute::fast_floor(0.5f*option->submap_lenth/option->resolution),compute::fast_floor(0.5f*option->submap_wide/option->resolution)),
min_(compute::fast_floor(-0.5f*option->submap_lenth/option->resolution),compute::fast_floor(-0.5f*option->submap_wide/option->resolution))
{
    lenth_=max_.x()-min_.x()+1;
    wide_=max_.y()-min_.y()+1;
    probability_cells_ = std::make_unique<std::vector<uint16_t>>(lenth_*wide_,unknow_value); 
    probability_cells_->resize(lenth_*wide_,0);
}

// @todo 这里inline真的好吗
inline uint16_t grid2d::Get_Probability(int x,int y)
{
    if(known_cells_box_.contains(Eigen::Vector2i(x, y)))
    {
        return (*probability_cells_)[Get_index(x,y)];
    }
    else return unknow_value;
}

//给ceres用的
inline uint16_t grid2d::Get_ProbabilityUncheck(int x,int y)
{
    return (*probability_cells_)[Get_index(x,y)];

}

//不进行溢出检查 请首先检查know_box
inline int grid2d::Get_index(int x,int y)
{
    return (y-min_.y()) *lenth_ + x-min_.x();
}
//扩张之前一定要想清楚 这里会按传入的点每次扩张2倍
void grid2d::extend(Eigen::Vector2i& min,Eigen::Vector2i& max)
{
   // constexpr static float edge_offset = 1e-6;//@TODO 这里到底是怎么边界判断
   // min += edge_offset;
   // max += edge_offset;
    Eigen::AlignedBox2i current_box(min_, max_);

    if( !current_box.contains(min)|| !current_box.contains(max))
    {
        int extend_lenth=lenth_;
        int extend_wide = wide_;

        Eigen::Vector2i current_min(-lenth_,-wide_);//自动先乘2
        Eigen::Vector2i current_max(lenth_,wide_);
        
        while (current_max.x() < max.x() || current_max.y() < max.y() || 
               current_min.x() > min.x() || current_min.y() > min.y()) 
        {
            current_max = current_max.array() * 2;
            current_min = current_min.array() * 2;
        }
        uint32_t new_lenth = current_max.x()- current_min.x()+1;
        uint32_t new_wide = current_max.y()- current_min.y()+1;
        auto new_grid= std::make_unique<std::vector<uint16_t>>();
        
        const int x_offset = min_.x() - current_min.x(); //倍增以后的偏移
        const int y_offset = min_.y() - current_min.y(); //倍增以后的偏移  请避免除2

        new_grid->resize(new_lenth*new_wide,unknow_value);
        for (size_t i = 0; i < wide_; ++i)
        {
            memcpy( &((*new_grid)[(y_offset+i)*new_lenth + x_offset]),&((*probability_cells_)[i*lenth_]),lenth_*sizeof(uint16_t));//这样可以保证前后get index的一致性
        }
        
        probability_cells_ = std::move(new_grid);
        lenth_ = new_lenth;
        wide_ = new_wide;
        max_ = current_max;
        min_ = current_min;
    }
}

}