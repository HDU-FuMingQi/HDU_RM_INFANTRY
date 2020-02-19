#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

/******拨盘,控制逻辑与云台类似(遥控器)*********/

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

int32_t Angle_Clac(moto_measure_t *moto);
void PWM_Set_Shootspeed(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty);
void Moto_Speed_Set(void);
void REVOLVER_Rc_Ctrl(void);
int get_signal();
void shoot_single();
void shoot_continuity();
void shoot_stop();


/*******键盘模式************/
/////////拨盘的键盘模式/////////
void REVOLVER_Key_Ctrl(void);

/******底盘键盘模式各类模式小函数*******/
void SHOOT_NORMAL_Ctrl(void);
void SHOOT_SINGLE_Ctrl(void);
void SHOOT_TRIPLE_Ctrl(void);
void SHOOT_HIGHTF_LOWS_Ctrl(void);
void SHOOT_MIDF_HIGHTS_Ctrl(void);
void SHOOT_AUTO_Ctrl(void);
void SHOOT_BUFF_Ctrl_Gimbal(void);

//视觉
bool GIMBAL_IfManulHit(void);//手动打符


#endif



