#ifndef CHASSIStaskH
#define CHASSIStaskH
#include "main.h"
typedef enum
{
	CHASSIS_FOLLOW_GIMBAL = 0,	//底盘跟随云盘行走
	CHASSIS_GYROSCOPE = 1,			//小陀螺模式
	CHASSIS_NORMAL   = 2,//底盘不跟随云台行走
	CHASSIS_CORGI    = 3,//扭屁股模式
	CHASSIS_ROSHAN   = 4,//打符模式
	CHASSIS_SLOW     = 5,//补弹低速模式
	CHASSIS_SZUPUP   = 6,//爬坡模式
	CHASSIS_MISS     = 7,//自动闪避模式
	CHASSIS_PISA     = 8,//45°模式
	
}eChassisAction;
extern eChassisAction actChassis;

//底盘模式选择
typedef enum
{
	CHASSIS_MECH_MODE = 0,//机械
	CHASSIS_GYRO_MODE = 1,//陀螺仪,底盘跟随云台
	
} eChassisCtrlMode;
extern eChassisCtrlMode  modeChassis;

typedef struct
{
	float vx;
	float vy;
	float vw;

} Chassis_Speed;

extern Chassis_Speed absolute_chassis_speed;
void SetChassisMotorMaxCurrent(const int16_t max1,const int16_t max2,const int16_t max3,const int16_t max4);
void RemoteControlChassis(void);
void KeyboardControlChassis(void);
void GetEnvironmentChassisMode(void);
void LimitChassisMotorCurrent(void);
void mecanum_calc(Chassis_Speed *speed, int16_t* out_speed);
void Mecanum_Set_Motor_Speed(int16_t*out_speed ,moto_measure_t* Motor );
void Absolute_Cal(Chassis_Speed* absolute_speed , float angle )	;
float FindMinAnglePNY(void);
float FindMinAngleFortyFive(void);
/*****************底盘功率*************************/
void Chassis_Power_Limit(void);
////////////////底盘启动状态/////////////////
void CHASSIS_REST(void);
/*****************键盘模式*************************/
/////////////////键盘底盘参数初始化//////////////////
void CHASSIS_InitArgument(void);
////////////////键盘控制底盘移动//////////////////
void CHAS_Key_Ctrl(void);

////////////////底盘键盘模式选择,按键响应////////////
void Chassis_NORMAL_Mode_Ctrl(void);
///////////////////鼠标控制底盘旋转,键盘QEC控制快速转圈/////////////
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax );

/////////////////键盘模式下底盘运动计算//////////////////////////////
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp );
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec ); //键盘模式辅助函数

/**************各种模式函数********************/
///////////////////扭屁股模式(位置不变版)//////////////////////
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp);
//////////////////手动爬坡模式////////////////////
void CHASSIS_SZUPUP_Mode_Ctrl(void);
/////////////////45°模式//////////////////////////
void CHASSIS_PISA_Mode_Ctrl(void);
//////////////////小陀螺模式/////////////////
void CHASSIS_GYROSCOPE_Mode_Ctrl(int16_t sMoveMax,int16_t sMoveRamp);

#endif



