#ifndef _FILTER_HPP_
#define _FILTER_HPP_

typedef struct vox_options
{
    int max_lenth;
    int min_lenth;//最小体素长  我说句实话二分肯定慢 不如就体素一次

    int vox_lenth;//体素滤波长  一般都是 1/分辨率
    float resolution;//分辨率
}Voxoptions;

Eigen::Vector2f volex_filter(const std::vector<Eigen::Vector2f>& point_cloud,Eigen::AlignedBox2i cells_box,float resolution);

#endif