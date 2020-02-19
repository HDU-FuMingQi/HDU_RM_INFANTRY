#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

/******����,�����߼�����̨����(ң����)*********/

typedef enum  //����ģʽ
{
	stop = 0,
	single = 1,
	continuity = 2,
}AMMUNITION_MODE;

typedef enum  //��������״̬
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


/*******����ģʽ************/
/////////���̵ļ���ģʽ/////////
void REVOLVER_Key_Ctrl(void);

/******���̼���ģʽ����ģʽС����*******/
void SHOOT_NORMAL_Ctrl(void);
void SHOOT_SINGLE_Ctrl(void);
void SHOOT_TRIPLE_Ctrl(void);
void SHOOT_HIGHTF_LOWS_Ctrl(void);
void SHOOT_MIDF_HIGHTS_Ctrl(void);
void SHOOT_AUTO_Ctrl(void);
void SHOOT_BUFF_Ctrl_Gimbal(void);

//�Ӿ�
bool GIMBAL_IfManulHit(void);//�ֶ����


#endif



