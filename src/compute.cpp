#include "compute.hpp"
namespace compute
{
    //用p计算odds
    inline float Odds(float p)
    {//float精度足够
       return p/(1.f-p);
    }

    inline float Odds2P(float odds)
    {
        return odds / (odds + 1.f);
    } 
    //0.1~0.9浮点转到[1,32767]
    uint16_t BoundFloat2Uint15(float input)//到时候实际概率就这么算
    {
        std::clamp(input,0.1f,0.9f);//规范到这之间
        return  std::clamp( static_cast<int>(((input-0.1f)/0.8f)*32766 +1),1,32767 );//防止浮点误差
    } 
    inline float BoundUint15tofloat(uint16_t input)//[1,32767]
    {
        return  std::clamp( static_cast<float>(((input-1)/32766)*0.8f+ 0.1),0.1f,0.9f);//防止浮点误差
    } 
}