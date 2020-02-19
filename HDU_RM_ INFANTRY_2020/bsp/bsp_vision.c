#include "bsp_vision.h"
#include "string.h"
VisionValueType VisionValue;
uint8_t vision_buf[VISION_BUFLEN];
void vision_uart_init(void)
{
	/* open uart idle it */


	uart_receive_dma_no_it(&VISION_HUART, vision_buf, VISION_MAX_LEN);

	__HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);
	__HAL_UART_ENABLE_IT(&VISION_HUART, UART_IT_IDLE);
}
void vision_callback_handler(uint8_t *buff)
{
	if(buff[0]=='s'&&buff[9]=='e')
	{
		memcpy(&(VisionValue.vision_yaw_value.buff[0]),&buff[1],4);
		memcpy(&(VisionValue.vision_pitch_value.buff[0]),&buff[5],4);
	}
	VisionValue.vision_pitch_value.value=VisionValue.vision_pitch_value.value/100.0;
	VisionValue.vision_yaw_value.value=VisionValue.vision_yaw_value.value/100.0;
	if(ISVISIONAUTO&&YawGimbalMode==USEENCODER)	//开启自瞄
	{
		Gimbal_MotorYaw.motor_value->target_angle=((float)Gimbal_MotorYaw.motor_value->angle-SENSITIVITY_VISION_GIMBAL_YAW_ENCODER*(VisionValue.vision_yaw_value.value));//目标编码器角度
		AngleLoop_f(&(Gimbal_MotorYaw.motor_value->target_angle),8192);
	}
//	else if(ISVISIONAUTO&&YawGimbalMode==USEIMU)	//开启自瞄
//	{
//		Target_Gimbal_Yaw_Position=((float)Gimbal_MotorYaw.motor_value->angle-VisionValue.vision_yaw_value.value);//目标编码器角度
//	}	
	if(ISVISIONAUTO&&PitchGimbalMode==USEENCODER)	//开启自瞄
	{
		Gimbal_MotorPitch.motor_value->target_angle=((float)Gimbal_MotorPitch.motor_value->angle+SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER*(VisionValue.vision_pitch_value.value));//目标编码器角度
		LimtValue_f(&(Gimbal_MotorPitch.motor_value->target_angle),GIMBAL_PITCH_ENCODER_MAX,GIMBAL_PITCH_ENCODER_MIN);
	}
//	else if(ISVISIONAUTO&&PitchGimbalMode==USEIMU)	//开启自瞄
//	{
//		Target_Gimbal_Pitch_Position=((float)Gimbal_MotorPitch.motor_value->angle-VisionValue.vision_pitch_value.value);
//	}	
}

/**
  * @brief  获取yaw误差像素，桥头吊射基地专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正，摄像头在云台   /************不知道640是干啥的**************
  */
void Vision_Base_Yaw_Pixel(float *error)
{
	if(VisionValue.vision_yaw_value.value != 0)
	{
		//输出为负时云台右移,为正时左移
		*error = -(VisionValue.vision_yaw_value.value- 640);
	}
	else
	{
		*error = 0;
	}
}
/*-----------------------------------------------------------------*/
	#define	COMPENSATION_YAW	0
	#define	COMPENSATION_PITCH	0
	#define COMPENSATION_PITCH_DIST 0
	float SB_K_comps = 3.f;

//角度初始化补偿
float Vision_Comps_Yaw   = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;//固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST;//根据距离补偿

/**
  * @brief  获取yaw误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = (-VisionValue.vision_yaw_value.value+ Vision_Comps_Yaw * VisionValue.distance/100) * 20;
//				* 8192.0f / 360.0f / 10.0f;//请根据自己对欧拉角的放大倍数来乘对应倍数
	if(VisionValue.vision_yaw_value.value == 0)//发零
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
float kvision_mouse_pitch = 0.007;
float mouse_pitch_comps = 0;//距离很远时开放鼠标补偿
float vision_pitch_dist = 2;//固定距离,超过此距离开启距离补偿
float vision_pitch_dist_far = 4.4f;//超过此距离开放鼠标补偿
void Vision_Error_Angle_Pitch(float *error)
{	
	
//	if(GIMBAL_AUTO_PITCH_SB() == TRUE)
//	{
//		*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch/SB_K_comps * VisionRecvData.distance/100)* 8192.0f / 360.0f / 10.0f;//因为pitch是机械模式,所以把欧拉角转换成机械角
//	}
//	else if(VisionRecvData.distance/100 >= vision_pitch_dist_far)
//	{
//		mouse_pitch_comps += MOUSE_Y_MOVE_SPEED * kvision_mouse_pitch;//注意正负
//		//限幅，防止太大
//		mouse_pitch_comps = constrain_float(mouse_pitch_comps, -3, 0);
//		*error = (VisionRecvData.pitch_angle
//						+ Vision_Comps_Pitch * VisionRecvData.distance/100
//							+ mouse_pitch_comps
//				 )
//				 * 8192.0f / 360.0f / 10.0f;//因为pitch是机械模式,所以把欧拉角转换成机械角		
//		
//	}
//	else
//	{
//		*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch * VisionRecvData.distance/100)* 8192.0f / 360.0f / 10.0f;//因为pitch是机械模式,所以把欧拉角转换成机械角
//	}
//	
//	if(VisionRecvData.pitch_angle == 0)
//	{
//		*error = 0;
//	}
}

/*-----------------------------------------------------------------*/

/**
  * @brief  获取距离
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Get_Distance(float *distance)
{
	*distance = VisionValue.distance;
	if(VisionValue.distance < 0)
	{
		*distance = 0;
	}
}


//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Get_New_Data = FALSE;

/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}




