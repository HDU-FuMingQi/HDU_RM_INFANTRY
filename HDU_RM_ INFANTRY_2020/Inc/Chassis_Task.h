#ifndef CHASSIStaskH
#define CHASSIStaskH
#include "main.h"
typedef enum
{
	CHASSIS_FOLLOW_GIMBAL = 0,	//���̸�����������
	CHASSIS_GYROSCOPE = 1,			//С����ģʽ
	CHASSIS_NORMAL   = 2,//���̲�������̨����
	CHASSIS_CORGI    = 3,//Ťƨ��ģʽ
	CHASSIS_ROSHAN   = 4,//���ģʽ
	CHASSIS_SLOW     = 5,//��������ģʽ
	CHASSIS_SZUPUP   = 6,//����ģʽ
	CHASSIS_MISS     = 7,//�Զ�����ģʽ
	CHASSIS_PISA     = 8,//45��ģʽ
	
}eChassisAction;
extern eChassisAction actChassis;

//����ģʽѡ��
typedef enum
{
	CHASSIS_MECH_MODE = 0,//��е
	CHASSIS_GYRO_MODE = 1,//������,���̸�����̨
	
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
/*****************���̹���*************************/
void Chassis_Power_Limit(void);
////////////////��������״̬/////////////////
void CHASSIS_REST(void);
/*****************����ģʽ*************************/
/////////////////���̵��̲�����ʼ��//////////////////
void CHASSIS_InitArgument(void);
////////////////���̿��Ƶ����ƶ�//////////////////
void CHAS_Key_Ctrl(void);

////////////////���̼���ģʽѡ��,������Ӧ////////////
void Chassis_NORMAL_Mode_Ctrl(void);
///////////////////�����Ƶ�����ת,����QEC���ƿ���תȦ/////////////
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax );

/////////////////����ģʽ�µ����˶�����//////////////////////////////
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp );
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec ); //����ģʽ��������

/**************����ģʽ����********************/
///////////////////Ťƨ��ģʽ(λ�ò����)//////////////////////
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp);
//////////////////�ֶ�����ģʽ////////////////////
void CHASSIS_SZUPUP_Mode_Ctrl(void);
/////////////////45��ģʽ//////////////////////////
void CHASSIS_PISA_Mode_Ctrl(void);
//////////////////С����ģʽ/////////////////
void CHASSIS_GYROSCOPE_Mode_Ctrl(int16_t sMoveMax,int16_t sMoveRamp);

#endif



