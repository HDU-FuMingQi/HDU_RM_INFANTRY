/*
 * Motor_Task.c
 *
 *  Created on: Nov 25, 2019
 *      Author: Tongw
 */
#include "bsp_motor.h"

Motortype Chassis_Motor1;
Motortype Chassis_Motor2;
Motortype Chassis_Motor3;
Motortype Chassis_Motor4;

Motortype Gimbal_MotorYaw;
Motortype Gimbal_MotorPitch;

Motortype Ammunition_Motor;

Motortype hero_Fire_Motor;
Motortype hero_friction_Motor_l;
Motortype hero_friction_Motor_r;

void Motor_Init(Motortype*motor,int ID,const float pid1[3],const float outmax1,const float imax1,
		const float pid2[3],const float outmax2,const float imax2)
{
	motor->Motor_PID_Position.I_Separation=3e38;	//积分分离
	motor->Motor_PID_Position.Dead_Zone=0;		//死区值
	motor->Motor_PID_Position.gama=0;			//微分先行滤波
	motor->Motor_PID_Position.angle_max=8192;
	motor->Motor_PID_Position.angle_min=0;

	motor->Motor_PID_Speed.I_Separation=3e38;	//积分分离
	motor->Motor_PID_Speed.Dead_Zone=0;		//死区值
	motor->Motor_PID_Speed.gama=0.1;			//微分先行滤波
	motor->Motor_PID_Speed.angle_max=0;
	motor->Motor_PID_Speed.angle_min=0;

	motor->ID=ID;
	motor->motor_value=&moto_CAN[ID-1];
	PID_Init(&(motor->Motor_PID_Position),PID_POSITION,pid1,outmax1,imax1);
	PID_Init(&(motor->Motor_PID_Speed),PID_POSITION,pid2,outmax2,imax2);
}
void Motor_Init2(Motortype*motor,int ID,const float pid1[3],const float outmax1,const float imax1,
		const float pid2[3],const float outmax2,const float imax2)
{
	motor->Motor_PID_Position.I_Separation=3e38;	//积分分离
	motor->Motor_PID_Position.Dead_Zone=0;		//死区值
	motor->Motor_PID_Position.gama=0;			//微分先行滤波
	motor->Motor_PID_Position.angle_max=8192;
	motor->Motor_PID_Position.angle_min=0;

	motor->Motor_PID_Speed.I_Separation=3e38;	//积分分离
	motor->Motor_PID_Speed.Dead_Zone=0;		//死区值
	motor->Motor_PID_Speed.gama=0.1;			//微分先行滤波
	motor->Motor_PID_Speed.angle_max=0;
	motor->Motor_PID_Speed.angle_min=0;

	motor->ID=ID;
	motor->motor_value=&moto_CAN2[ID-1];
	PID_Init(&(motor->Motor_PID_Position),PID_POSITION,pid1,outmax1,imax1);
	PID_Init(&(motor->Motor_PID_Speed),PID_POSITION,pid2,outmax2,imax2);
}
/**
  * @brief  获取6020电机角速度
  * @param  void
  * @retval void
  * @attention 
  */
void Get6020SpeedCallback(void const * argument)
{
	Cal_6020_speed_dp10ms(Gimbal_MotorYaw.ID);
	Cal_6020_speed_dp10ms(Gimbal_MotorPitch.ID);
	Cal_6020_speed_dp10ms(hero_Fire_Motor.ID);
}

