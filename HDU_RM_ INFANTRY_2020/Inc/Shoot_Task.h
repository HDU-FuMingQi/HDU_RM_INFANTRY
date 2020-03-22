#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

/******拨盘,控制逻辑与云台类似(遥控器)*********/

typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;
extern eRevolverCtrlMode Revolver_mode;

typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;
extern eShootAction actShoot;

typedef enum  //拨弹模式
{
	stop = 0,
	single = 1,
	continuity = 2,
}AMMUNITION_MODE;

typedef enum  //单发发射状态
{
	unstart = 0,
	start = 1,
	finish = 2,
}SHOOT_STATE;

extern int8_t ammunition_mode;
extern int32_t total_angle_next;
extern uint8_t shoot_state;
extern double initial_speed;


void GetEnvironmentShootMode(void);  //选择模式射击

/***********************遥控模式****************************/
					void REVOLVER_Rc_Ctrl(void);
					void RemoteShootSpeedSet(void);

			/******拨盘遥控模式各类模式小函数*******/
					void shoot_single();
					void shoot_continuity();
					void shoot_stop();

			/**************（键盘的射击写在摩擦轮里）**************/
					void PWM_Set_Shootspeed(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty);
					int get_signal();
/******************************************************************/


/*******************键盘模式******************************/
				void REVOLVER_Key_Ctrl(void);  //拨盘的键盘选择模式
				void KeyboardShootSpeedSet(void);  //拨盘的键盘射击控制

	   /******拨盘键盘模式各类模式小函数*******/
				void SHOOT_NORMAL_Ctrl(void);
				void SHOOT_SINGLE_Ctrl(void);
				void SHOOT_TRIPLE_Ctrl(void);
				void SHOOT_HIGHTF_LOWS_Ctrl(void);
				void SHOOT_MIDF_HIGHTS_Ctrl(void);
				void SHOOT_AUTO_Ctrl(void);
				void SHOOT_BUFF_Ctrl_Gimbal(void);           /***/

				void REVOLVER_KeySpeedCtrl(void);   //键盘模式拨盘速度环控制    
				void REVOLVER_KeyPosiCtrl(void);  //键盘模式拨盘位置环控制      /***核心发射函数***/


	  /****拨盘电机数据更新,CAN2中断中调用****/
				void REVOLVER_UpdateMotorAngle();
				void REVOLVER_UpdateMotorSpeed( int16_t speed );
				void REVOL_UpdateMotorAngleSum( void );             //    ***

		 /*****PID控制*******/
				void REVOL_SpeedLoop( void );     //速度环
				void REVOL_PositionLoop( void );  //位置环

			/****卡弹处理*****/
				void REVOL_SpeedStuck(void);
				void REVOL_PositStuck(void);

			/******射频热量限制******/
				bool Revolver_Heat_Limit(void);

			/*************************************/
				void REVOLVER_StopMotor(void);      //拨盘失控保护
				void REVOLVER_InitArgument(void);   //拨盘参数初始化
				void REVOLVER_Rest();            //发弹清零,哪怕鼠标继续点击也不给发弹
				void Revolver_Angle_Rest(void);   //拨盘角度清零
/******************************************************************/


/*************************************/
portTickType REVOL_uiGetRevolTime(void);   //点射时间获取

#endif



