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
	
	float     distance;			//距离
	uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是	
	uint8_t   identify_buff;	//打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
	uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
	
}VisionValueType;
extern VisionValueType VisionValue;
extern uint8_t vision_buf[];
void vision_callback_handler(uint8_t *buff);
void vision_uart_init(void);

/*****视觉偏差获取******/
//////////获取yaw误差像素，桥头吊射基地专用////////////////
void Vision_Base_Yaw_Pixel(float *error);
///////////////获取yaw误差角度，自瞄专用/////////////////
void Vision_Error_Angle_Yaw(float *error);
//////////////获取pitch误差角度，自瞄专用///////////////////
void Vision_Error_Angle_Pitch(float *error);

///////////////获取距离/////////////////////////
void Vision_Get_Distance(float *distance);

/********视觉辅助函数*********/
/////////判断视觉数据更新了吗/////////
bool Vision_If_Update(void);
extern uint8_t Vision_Get_New_Data;
////////视觉数据更新标志位手动置0//////////
void Vision_Clean_Update_Flag(void);


#endif


