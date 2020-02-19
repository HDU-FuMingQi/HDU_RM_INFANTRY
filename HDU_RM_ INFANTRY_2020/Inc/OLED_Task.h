#ifndef OLED_TASK_H
#define OLED_TASK_H
#include "main.h"
#include "bsp_oled.h"
extern void OLED_Change_Param(void);
extern void Oled_ShowIMU(void);
extern void Oled_Show_Manu(void);
extern void OLED_Show_Info(void);
extern void OLED_Change_Param(void);
extern void Oled_ShowMotorSet(void);
extern void Oled_ShowJY901(void);
extern void Oled_ShowRC(void);
extern void Oled_ShowRC_PC(void);
extern void Oled_ShowMotor(void);
extern void oled_LOGO(void);

extern void Oled_ShowGM6020Set(void);
extern void Oled_ShowGM3508Set_l(void);
extern void Oled_ShowGM3508Set_r(void);

#endif



