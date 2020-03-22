#ifndef _FRICTION_H
#define _FRICTION_H

#include "main.h"

/*********摩擦轮主控************/
void FRICTION_Ctrl();  //摩擦轮控制函数
void FRIC_KeyLevel_Ctrl(void); //摩擦轮等级控制,键盘专用,根据等级自动设置射速

/****摩擦轮辅助函数*****/
void Friction_Ramp(void);     //摩擦轮输出斜坡
uint16_t Fric_GetHeatInc(void);  //当前PWM对应单发子弹热量增加值(射速)
float Fric_GetSpeedReal(void);  //获取当前摩擦轮PWM输出值


void TIM_FrictionPwmOutp(int16_t pwm1,int16_t pwm2);   //摩擦轮输出函数

#endif
