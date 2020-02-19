#ifndef __bsp_pid
#define __bsp_pid
#include "myType.h"
#include "stm32f4xx_hal.h"
#include "main.h"



void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_Calc(PidTypeDef *pid, const fp32 ref, const fp32 set);
void PID_clear(PidTypeDef *pid);
//fp32 PID_Calc_Angle(PidTypeDef *pid, fp32 ref, fp32 set);
#endif
