#ifndef GIMBALTASKH
#define GIMBALTASKH
#include "main.h"


#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1

#define NOW  0
#define LAST 1

//��̨ģʽѡ��
typedef enum
{
	CLOUD_MECH_MODE = 0,
	CLOUD_GYRO_MODE = 1,
} eGimbalCtrlMode;
extern eGimbalCtrlMode  modeGimbal;


/* ��̨����ģʽ:
   
   ��ͨ             	NORMAL
   ��ͷ180��             AROUND
   ���             	BUFF
   ����,pitchˮƽ   	LEVEL
   ��еģʽpitcḩͷ	HIGH
   ����Ťͷ90��          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//����ģʽ,����ģʽѡ��
	GIMBAL_AROUND  = 1,//180���ͷ
	GIMBAL_BUFF    = 2,//���ģʽ,��
	GIMBAL_LEVEL   = 3,//���ֿ���,��̨ˮƽ
	GIMBAL_MANUAL  = 4,//�ֶ����ģʽ
	GIMBAL_SM_BUFF = 5,//С��
	GIMBAL_TURN    = 7,//90��Ťͷ
	GIMBAL_AUTO    = 8,//����
	GIMBAL_BASE    = 9,//��ͷ�������
	
}eGimbalAction;
extern eGimbalAction  actGimbal;




typedef enum
{
	USEENCODER,
	USEIMU
}GimbalModeType;
extern GimbalModeType YawGimbalMode ;
extern GimbalModeType PitchGimbalMode ;

extern float Target_Gimbal_Yaw_Position;				//ʹ��IMUʱ��Ŀ��ֵ
extern float Target_Gimbal_Pitch_Position;			//ʹ��IMUʱ��Ŀ��ֵ
void Cal_6020_speed_dp10ms(int ID);
void GimbalOpenInit(void);
void RemoteControlGimbal(void);
void KeyboardControlGimbal(void);
void AngleLoop (float* angle ,float max);
void AngleLoop_f (float* angle ,float max);

//////////�Ƿ������ģʽ//////////
bool GIMBAL_IfBuffHit(void);
////////////��ȡ�����ƶ�ģʽ//////////
bool CHASSIS_IfActiveMode(void);


typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
}speed_calc_data_t;
/***********����Ĳ����Ǽ���ģʽ*******************/
///////////��̨������ʼ��/////////////
void GIMBAL_InitArgument(void);
///////////////���̿�����̨ģʽ//////////////
void GIMBAL_Key_Ctrl(void);

/*******************��̨����ģʽ����ģʽС����*******************/
//////////////��̨����ģʽѡ��,������Ӧ/////////////
void GIMBAL_NORMAL_Mode_Ctrl(void);

////////////����ģʽ////////////////
void GIMBAL_LEVEL_Mode_Ctrl(void);
//////////������ƺ���///////////
void GIMBAL_AUTO_Mode_Ctrl(void);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
//////////��ͷ����ģʽ/////////////
void GIMBAL_BASE_Mode_Ctrl(void);
///////////���ģʽ������ͷλ����̨/////////
void GIMBAL_BUFF_Mode_Ctrl_Gimbal(void);

/*******************�Ƿ�*******************/
//////////�Ƿ��������ڱ�////////////
bool GIMBAL_AUTO_PITCH_SB(void);
////////yaw�Ὺ��Ԥ���ʱ����̨�Ƿ�λ////////////
bool GIMBAL_MOBPRE_YAW_FIRE(void);
////////����yaw��Ԥ���Ƿ��Ѿ�����////////////
bool GIMBAL_IfAuto_MobPre_Yaw(void);
#endif



