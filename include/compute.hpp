#ifndef _COMPUTE_HPP_
#define _COMPUTE_HPP_
#include <stdint.h>
#include <algorithm> 
namespace compute
{
inline float Odds(float p);
inline float Odds2P(float odds);

//0.1~0.9浮点转到[1,32767]
uint16_t BoundFloat2Uint15(float input);
inline float BoundUint15tofloat(uint16_t input);//[1,32767]

}//namespace
#endif