#ifndef _SETTINGS_HPP_
#define _SETTINGS_HPP_
#define unknow_value 0  //我们设计点最低的可信度为1，也就是说值为0的时候是未知点
#define SCOUT_OMP_THREAD_NUM_FRONT 2//前端线程数，不建议太多
#define Point_Unknow Eigen::Vector2f{NAN, NAN}
#endif