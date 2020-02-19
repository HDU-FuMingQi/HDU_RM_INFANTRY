#ifndef _bspvision_h
#define _bspvision_h
#include "main.h"
#define VISION_HUART huart7
#define VISION_MAX_LEN 50
#define VISION_BUFLEN 10
typedef struct{
	struct
	{
		uint8_t buff[4];
		float value;
	}vision_yaw_value;
	struct
	{
		uint8_t buff[4];
		float value;
	}vision_pitch_value;
	
	float     distance;			//����
	uint8_t   centre_lock;		//�Ƿ���׼�����м�  0û��  1��׼����
	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��	
	uint8_t   identify_buff;	//���ʱ�Ƿ�ʶ����Ŀ�꣬1�ǣ�2ʶ���л���װ�ף�0ûʶ��
	uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
	
}VisionValueType;
extern VisionValueType VisionValue;
extern uint8_t vision_buf[];
void vision_callback_handler(uint8_t *buff);
void vision_uart_init(void);

/*****�Ӿ�ƫ���ȡ******/
//////////��ȡyaw������أ���ͷ�������ר��////////////////
void Vision_Base_Yaw_Pixel(float *error);
///////////////��ȡyaw���Ƕȣ�����ר��/////////////////
void Vision_Error_Angle_Yaw(float *error);
//////////////��ȡpitch���Ƕȣ�����ר��///////////////////
void Vision_Error_Angle_Pitch(float *error);

///////////////��ȡ����/////////////////////////
void Vision_Get_Distance(float *distance);

/********�Ӿ���������*********/
/////////�ж��Ӿ����ݸ�������/////////
bool Vision_If_Update(void);
extern uint8_t Vision_Get_New_Data;
////////�Ӿ����ݸ��±�־λ�ֶ���0//////////
void Vision_Clean_Update_Flag(void);


#endif


