#ifndef _FRICTION_H
#define _FRICTION_H

#include "main.h"

/*********Ħ��������************/
void FRICTION_Ctrl();  //Ħ���ֿ��ƺ���
void FRIC_KeyLevel_Ctrl(void); //Ħ���ֵȼ�����,����ר��,���ݵȼ��Զ���������

/****Ħ���ָ�������*****/
void Friction_Ramp(void);     //Ħ�������б��
uint16_t Fric_GetHeatInc(void);  //��ǰPWM��Ӧ�����ӵ���������ֵ(����)
float Fric_GetSpeedReal(void);  //��ȡ��ǰĦ����PWM���ֵ


void TIM_FrictionPwmOutp(int16_t pwm1,int16_t pwm2);   //Ħ�����������

#endif
