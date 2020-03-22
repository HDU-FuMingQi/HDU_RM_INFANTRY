#ifndef _CONTROL_H
#define _CONTROL_H

#include "main.h"

//float类型绝对值函数
float abs_float(float a);

float constrain_float(float amt, float low, float high);

//斜坡函数
float RAMP_float( float final, float now, float ramp );
float RampInc_float( float *buffer, float now, float ramp ); //使目标输出值缓慢等于指针输入值

void LimtValue_f(float* VALUE,float MAX,float MIN);
void LimtValue_16(int16_t* VALUE,int16_t MAX,int16_t MIN);
void LimtValue_u16(uint16_t* VALUE,uint16_t MAX,uint16_t MIN);

/****************角度限制函数*******************/
void AngleLoop (float* angle ,float max);
void AngleLoop_f (float* angle ,float max);

#endif


