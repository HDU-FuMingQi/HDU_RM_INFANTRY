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
	if(ISVISIONAUTO&&YawGimbalMode==USEENCODER)	//��������
	{
		Gimbal_MotorYaw.motor_value->target_angle=((float)Gimbal_MotorYaw.motor_value->angle-SENSITIVITY_VISION_GIMBAL_YAW_ENCODER*(VisionValue.vision_yaw_value.value));//Ŀ��������Ƕ�
		AngleLoop_f(&(Gimbal_MotorYaw.motor_value->target_angle),8192);
	}
//	else if(ISVISIONAUTO&&YawGimbalMode==USEIMU)	//��������
//	{
//		Target_Gimbal_Yaw_Position=((float)Gimbal_MotorYaw.motor_value->angle-VisionValue.vision_yaw_value.value);//Ŀ��������Ƕ�
//	}	
	if(ISVISIONAUTO&&PitchGimbalMode==USEENCODER)	//��������
	{
		Gimbal_MotorPitch.motor_value->target_angle=((float)Gimbal_MotorPitch.motor_value->angle+SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER*(VisionValue.vision_pitch_value.value));//Ŀ��������Ƕ�
		LimtValue_f(&(Gimbal_MotorPitch.motor_value->target_angle),GIMBAL_PITCH_ENCODER_MAX,GIMBAL_PITCH_ENCODER_MIN);
	}
//	else if(ISVISIONAUTO&&PitchGimbalMode==USEIMU)	//��������
//	{
//		Target_Gimbal_Pitch_Position=((float)Gimbal_MotorPitch.motor_value->angle-VisionValue.vision_pitch_value.value);
//	}	
}

/**
  * @brief  ��ȡyaw������أ���ͷ�������ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������������ͷ����̨   /************��֪��640�Ǹ�ɶ��**************
  */
void Vision_Base_Yaw_Pixel(float *error)
{
	if(VisionValue.vision_yaw_value.value != 0)
	{
		//���Ϊ��ʱ��̨����,Ϊ��ʱ����
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

//�Ƕȳ�ʼ������
float Vision_Comps_Yaw   = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;//�̶���������С�����Ӱ��
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST;//���ݾ��벹��

/**
  * @brief  ��ȡyaw���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
	*error = (-VisionValue.vision_yaw_value.value+ Vision_Comps_Yaw * VisionValue.distance/100) * 20;
//				* 8192.0f / 360.0f / 10.0f;//������Լ���ŷ���ǵķŴ������˶�Ӧ����
	if(VisionValue.vision_yaw_value.value == 0)//����
	{
		*error = 0;
	}
}

/**
  * @brief  ��ȡpitch���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  �Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ
  */
float kvision_mouse_pitch = 0.007;
float mouse_pitch_comps = 0;//�����Զʱ������겹��
float vision_pitch_dist = 2;//�̶�����,�����˾��뿪�����벹��
float vision_pitch_dist_far = 4.4f;//�����˾��뿪����겹��
void Vision_Error_Angle_Pitch(float *error)
{	
	
//	if(GIMBAL_AUTO_PITCH_SB() == TRUE)
//	{
//		*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch/SB_K_comps * VisionRecvData.distance/100)* 8192.0f / 360.0f / 10.0f;//��Ϊpitch�ǻ�еģʽ,���԰�ŷ����ת���ɻ�е��
//	}
//	else if(VisionRecvData.distance/100 >= vision_pitch_dist_far)
//	{
//		mouse_pitch_comps += MOUSE_Y_MOVE_SPEED * kvision_mouse_pitch;//ע������
//		//�޷�����ֹ̫��
//		mouse_pitch_comps = constrain_float(mouse_pitch_comps, -3, 0);
//		*error = (VisionRecvData.pitch_angle
//						+ Vision_Comps_Pitch * VisionRecvData.distance/100
//							+ mouse_pitch_comps
//				 )
//				 * 8192.0f / 360.0f / 10.0f;//��Ϊpitch�ǻ�еģʽ,���԰�ŷ����ת���ɻ�е��		
//		
//	}
//	else
//	{
//		*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch * VisionRecvData.distance/100)* 8192.0f / 360.0f / 10.0f;//��Ϊpitch�ǻ�еģʽ,���԰�ŷ����ת���ɻ�е��
//	}
//	
//	if(VisionRecvData.pitch_angle == 0)
//	{
//		*error = 0;
//	}
}

/*-----------------------------------------------------------------*/

/**
  * @brief  ��ȡ����
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


//�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
uint8_t Vision_Get_New_Data = FALSE;

/**
  * @brief  �ж��Ӿ����ݸ�������
  * @param  void
  * @retval TRUE������   FALSEû����
  * @attention  Ϊ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Get_New_Data��TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  �Ӿ����ݸ��±�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}




