#ifndef bsp_motor_h
#define bsp_motor_h

#include "main.h"
#include "bsp_pid.h"
#include "mytype.h"


extern Motortype Chassis_Motor1;
extern Motortype Chassis_Motor2;
extern Motortype Chassis_Motor3;
extern Motortype Chassis_Motor4;

extern Motortype Gimbal_MotorYaw;
extern Motortype Gimbal_MotorPitch;

extern Motortype Ammunition_Motor;
	
extern Motortype hero_Fire_Motor;
extern Motortype hero_friction_Motor_l;
extern Motortype hero_friction_Motor_r;

void Motor_Init(Motortype*motor,int ID,const float pid1[3],const float outmax1,const float imax1,
		const float pid2[3],const float outmax2,const float imax2);
void Motor_Init2(Motortype*motor,int ID,const float pid1[3],const float outmax1,const float imax1,
		const float pid2[3],const float outmax2,const float imax2);
#endif
