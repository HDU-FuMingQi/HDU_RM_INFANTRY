#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

/******����,�����߼�����̨����(ң����)*********/

typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;
extern eRevolverCtrlMode Revolver_mode;

typedef enum
{
	SHOOT_NORMAL       =  0,//���ģʽѡ��,Ĭ�ϲ���
	SHOOT_SINGLE       =  1,//����
	SHOOT_TRIPLE       =  2,//������
	SHOOT_HIGHTF_LOWS  =  3,//����Ƶ������
	SHOOT_MIDF_HIGHTS  =  4,//����Ƶ������
	SHOOT_BUFF         =  5,//���ģʽ
	SHOOT_AUTO         =  6,//�����Զ����
}eShootAction;
extern eShootAction actShoot;

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


void GetEnvironmentShootMode(void);  //ѡ��ģʽ���

/***********************ң��ģʽ****************************/
					void REVOLVER_Rc_Ctrl(void);
					void RemoteShootSpeedSet(void);

			/******����ң��ģʽ����ģʽС����*******/
					void shoot_single();
					void shoot_continuity();
					void shoot_stop();

			/**************�����̵����д��Ħ�����**************/
					void PWM_Set_Shootspeed(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty);
					int get_signal();
/******************************************************************/


/*******************����ģʽ******************************/
				void REVOLVER_Key_Ctrl(void);  //���̵ļ���ѡ��ģʽ
				void KeyboardShootSpeedSet(void);  //���̵ļ����������

	   /******���̼���ģʽ����ģʽС����*******/
				void SHOOT_NORMAL_Ctrl(void);
				void SHOOT_SINGLE_Ctrl(void);
				void SHOOT_TRIPLE_Ctrl(void);
				void SHOOT_HIGHTF_LOWS_Ctrl(void);
				void SHOOT_MIDF_HIGHTS_Ctrl(void);
				void SHOOT_AUTO_Ctrl(void);
				void SHOOT_BUFF_Ctrl_Gimbal(void);           /***/

				void REVOLVER_KeySpeedCtrl(void);   //����ģʽ�����ٶȻ�����    
				void REVOLVER_KeyPosiCtrl(void);  //����ģʽ����λ�û�����      /***���ķ��亯��***/


	  /****���̵�����ݸ���,CAN2�ж��е���****/
				void REVOLVER_UpdateMotorAngle();
				void REVOLVER_UpdateMotorSpeed( int16_t speed );
				void REVOL_UpdateMotorAngleSum( void );             //    ***

		 /*****PID����*******/
				void REVOL_SpeedLoop( void );     //�ٶȻ�
				void REVOL_PositionLoop( void );  //λ�û�

			/****��������*****/
				void REVOL_SpeedStuck(void);
				void REVOL_PositStuck(void);

			/******��Ƶ��������******/
				bool Revolver_Heat_Limit(void);

			/*************************************/
				void REVOLVER_StopMotor(void);      //����ʧ�ر���
				void REVOLVER_InitArgument(void);   //���̲�����ʼ��
				void REVOLVER_Rest();            //��������,�������������Ҳ��������
				void Revolver_Angle_Rest(void);   //���̽Ƕ�����
/******************************************************************/


/*************************************/
portTickType REVOL_uiGetRevolTime(void);   //����ʱ���ȡ

#endif



