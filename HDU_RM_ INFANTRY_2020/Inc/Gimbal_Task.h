#ifndef GIMBALTASKH
#define GIMBALTASKH
#include "main.h"


#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1

#define NOW  0
#define LAST 1

//云台模式选择
typedef enum
{
	CLOUD_MECH_MODE = 0,
	CLOUD_GYRO_MODE = 1,
} eGimbalCtrlMode;
extern eGimbalCtrlMode  modeGimbal;


/* 云台操作模式:
   
   普通             	NORMAL
   调头180°             AROUND
   打符             	BUFF
   补弹,pitch水平   	LEVEL
   机械模式pitch抬头	HIGH
   快速扭头90°          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//正常模式,进行模式选择
	GIMBAL_AROUND  = 1,//180°调头
	GIMBAL_BUFF    = 2,//打符模式,大
	GIMBAL_LEVEL   = 3,//弹仓开启,云台水平
	GIMBAL_MANUAL  = 4,//手动打符模式
	GIMBAL_SM_BUFF = 5,//小符
	GIMBAL_TURN    = 7,//90°扭头
	GIMBAL_AUTO    = 8,//自瞄
	GIMBAL_BASE    = 9,//桥头吊射基地
	
}eGimbalAction;
extern eGimbalAction  actGimbal;




typedef enum
{
	USEENCODER,
	USEIMU
}GimbalModeType;
extern GimbalModeType YawGimbalMode ;
extern GimbalModeType PitchGimbalMode ;

extern float Target_Gimbal_Yaw_Position;				//使用IMU时的目标值
extern float Target_Gimbal_Pitch_Position;			//使用IMU时的目标值
void Cal_6020_speed_dp10ms(int ID);
void GimbalOpenInit(void);
void RemoteControlGimbal(void);
void KeyboardControlGimbal(void);
void AngleLoop (float* angle ,float max);
void AngleLoop_f (float* angle ,float max);

//////////是否开启打符模式//////////
bool GIMBAL_IfBuffHit(void);
////////////获取底盘移动模式//////////
bool CHASSIS_IfActiveMode(void);


typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;
/***********下面的部分是键盘模式*******************/
///////////云台参数初始化/////////////
void GIMBAL_InitArgument(void);
///////////////键盘控制云台模式//////////////
void GIMBAL_Key_Ctrl(void);

/*******************云台键盘模式各类模式小函数*******************/
//////////////云台键盘模式选择,按键响应/////////////
void GIMBAL_NORMAL_Mode_Ctrl(void);

////////////补弹模式////////////////
void GIMBAL_LEVEL_Mode_Ctrl(void);
//////////自瞄控制函数///////////
void GIMBAL_AUTO_Mode_Ctrl(void);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
//////////桥头吊射模式/////////////
void GIMBAL_BASE_Mode_Ctrl(void);
///////////打符模式，摄像头位于云台/////////
void GIMBAL_BUFF_Mode_Ctrl_Gimbal(void);

/*******************是否*******************/
//////////是否在自瞄哨兵////////////
bool GIMBAL_AUTO_PITCH_SB(void);
////////yaw轴开启预测的时候云台是否到位////////////
bool GIMBAL_MOBPRE_YAW_FIRE(void);
////////自瞄yaw轴预测是否已经开启////////////
bool GIMBAL_IfAuto_MobPre_Yaw(void);
#endif



