#ifndef _CONTROL_H
#define _CONTROL_H

#include "main.h"

//float���;���ֵ����
float abs_float(float a);

float constrain_float(float amt, float low, float high);

//б�º���
float RAMP_float( float final, float now, float ramp );
float RampInc_float( float *buffer, float now, float ramp ); //ʹĿ�����ֵ��������ָ������ֵ

void LimtValue_f(float* VALUE,float MAX,float MIN);
void LimtValue_16(int16_t* VALUE,int16_t MAX,int16_t MIN);
void LimtValue_u16(uint16_t* VALUE,uint16_t MAX,uint16_t MIN);

/****************�Ƕ����ƺ���*******************/
void AngleLoop (float* angle ,float max);
void AngleLoop_f (float* angle ,float max);

#endif


