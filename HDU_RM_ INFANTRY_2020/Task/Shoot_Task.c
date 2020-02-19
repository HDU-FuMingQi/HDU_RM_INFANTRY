#include "shoot_task.h"
int8_t ammunition_mode = stop;
int32_t total_angle_next;
uint8_t shoot_state = unstart;
double initial_speed = 10; //Ħ�����ٶ�
const static fp32 Ammunition_Motor_Position_pid[3] = {0.05, 0, 0};
const static fp32 Ammunition_Motor_Speed_pid[3] = {15, 0.01, 2};

void ShootFun(void const * argument)
{
	portTickType currentTime;
	Motor_Init2(&Ammunition_Motor, 1, Ammunition_Motor_Position_pid, 2000, 500, Ammunition_Motor_Speed_pid, 20000, 10000);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	while (1)
	{
		currentTime = xTaskGetTickCount();

		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)
		{
			
		}
		else
		{
			if (SYSTEM_GetRemoteMode() == RC)
			{
				REVOLVER_Rc_Ctrl();	
				Moto_Speed_Set();
			}
			else
			{
				REVOLVER_Key_Ctrl();
			}
		}
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS); //������ʱ
	}
}

void PWM_Set_Shootspeed(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty)
{

	switch (tim_channel)
	{
	case TIM_CHANNEL_1:
		tim->Instance->CCR1 = ((tim->Init.Period + 1) * duty) - 1;
		break;
	case TIM_CHANNEL_2:
		tim->Instance->CCR2 = ((tim->Init.Period + 1) * duty) - 1;
		break;
	case TIM_CHANNEL_3:
		tim->Instance->CCR3 = ((tim->Init.Period + 1) * duty) - 1;
		break;
	case TIM_CHANNEL_4:
		tim->Instance->CCR4 = ((tim->Init.Period + 1) * duty) - 1;
		break;
	}
}



void REVOLVER_Rc_Ctrl(void)
{
	switch (rc.sw2)
	{
	case 1:
		ammunition_mode = single;
		break;
	case 2:
		ammunition_mode = continuity;
		break;
	default:
		ammunition_mode = stop;
		break;
	}
}

int get_signal()
{
		return HAL_GPIO_ReadPin(key_signal_GPIO_Port,key_signal_Pin);
}

void Moto_Speed_Set()
{
	switch (ammunition_mode)
	{
		case single :
			if(shoot_state == unstart)
			{
			shoot_state = start;
			shoot_single();
			}
			break;
		case continuity:
			shoot_continuity();
		  break;
		default:
			shoot_stop();
		  shoot_state = unstart;
			break;
	}
	
}

void shoot_single()
{
	PWM_Set_Shootspeed(&htim2, TIM_CHANNEL_1,0.06);
	PWM_Set_Shootspeed(&htim2, TIM_CHANNEL_2,0.06);
	
	  int ammunition_flag= unstart;
		while(shoot_state != finish && rc.sw2 == 1)
		{
		
			if (get_signal() && ammunition_flag == finish) 
			{
			shoot_state = finish;
			Ammunition_Motor.motor_value->target_speed_rpm = 0;
			PID_Calc(&Ammunition_Motor.Motor_PID_Speed, Ammunition_Motor.motor_value->speed_rpm,
				 Ammunition_Motor.motor_value->target_speed_rpm);
	    set_moto1234_current(&hcan2, Ammunition_Motor.Motor_PID_Speed.out, 0, 0, 0);

			}
			else 
			{
				if(get_signal())
				{
					ammunition_flag += 1; 
				}
			HAL_Delay(10);
			Ammunition_Motor.motor_value->target_speed_rpm = -500;
			PID_Calc(&Ammunition_Motor.Motor_PID_Speed, Ammunition_Motor.motor_value->speed_rpm,
				 Ammunition_Motor.motor_value->target_speed_rpm);
	    set_moto1234_current(&hcan2, Ammunition_Motor.Motor_PID_Speed.out, 0, 0, 0);
				
				
			}
		}

	
}



void shoot_continuity()
{
	 PWM_Set_Shootspeed(&htim2, TIM_CHANNEL_1,0.07);
	 PWM_Set_Shootspeed(&htim2, TIM_CHANNEL_2,0.07);
	 Ammunition_Motor.motor_value->target_speed_rpm = -500;
	 PID_Calc(&Ammunition_Motor.Motor_PID_Speed, Ammunition_Motor.motor_value->speed_rpm,
				 Ammunition_Motor.motor_value->target_speed_rpm);
	 set_moto1234_current(&hcan2, Ammunition_Motor.Motor_PID_Speed.out, 0, 0, 0);

}


void shoot_stop()
{
  PWM_Set_Shootspeed(&htim2, TIM_CHANNEL_1,0.05);
	PWM_Set_Shootspeed(&htim2, TIM_CHANNEL_2,0.05);
	Ammunition_Motor.motor_value->target_speed_rpm = 0;
	PID_Calc(&Ammunition_Motor.Motor_PID_Speed, Ammunition_Motor.motor_value->speed_rpm,
				 Ammunition_Motor.motor_value->target_speed_rpm);
	set_moto1234_current(&hcan2, Ammunition_Motor.Motor_PID_Speed.out, 0, 0, 0);

}


/***********************************************************************************************
     *  ����Ĳ����Ǽ���ģʽ
			
				
           /**�Ƕ�ֵ�ܿ����ǲ��Ե�

 ****************************************************************************************************/

/******����,�����߼�����̨����(����)*********/

//���̵��ģʽ,λ�û����ٶȻ�
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//����ģʽѡ��
//���̵��ģʽ,λ�û����ٶȻ�
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;
eRevolverCtrlMode Revolver_mode;

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
eShootAction actShoot;


/*******����ģʽ************/

/**
  * @brief  ���̵ļ���ģʽ
  * @param  void
  * @retval void
  * @attention ������λ�û�����
  */
void REVOLVER_Key_Ctrl(void)
{
	Revolver_mode = REVOL_POSI_MODE;//��ֹ������,Ĭ��λ�û�
	
	SHOOT_NORMAL_Ctrl();//ȷ�����ģʽ
	
	/*- ȷ�������������ģʽ -*/
	switch(actShoot)
	{
		case SHOOT_NORMAL:
			//���ģʽѡ��,Ĭ�ϲ���
			SHOOT_NORMAL_Ctrl();
		break;
		
		case SHOOT_SINGLE:
			//��һ���������,��������
			SHOOT_SINGLE_Ctrl();
		break;
		
		case SHOOT_TRIPLE:
			//��������
			SHOOT_TRIPLE_Ctrl();
//			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_HIGHTF_LOWS:
			//B����Ƶ
			SHOOT_HIGHTF_LOWS_Ctrl();
		break;
		
		case SHOOT_MIDF_HIGHTS:
			//Z�Ƽ�
			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_BUFF:
			//����Զ���
			Revolver_mode = REVOL_POSI_MODE;
			SHOOT_BUFF_Ctrl_Gimbal();
		break;
		
		case SHOOT_AUTO:
			//�Ҽ�����ʱ�Զ���
//			SHOOT_AUTO_Ctrl();
		break;		
	}
	
//	/*- ��ʼ����,������ -*/
//	if(Revolver_mode == REVOL_SPEED_MODE && Fric_GetSpeedReal() > REVOL_CAN_OPEN)
//	{
//		REVOLVER_KeySpeedCtrl();
//	}
//	else if(Revolver_mode == REVOL_POSI_MODE && Fric_GetSpeedReal() > REVOL_CAN_OPEN)
//	{
//		REVOLVER_KeyPosiCtrl();
//	}
}

/************************���̼���ģʽ����ģʽС����****************************/
/**
  * @brief  ����ģʽ�·���ģʽѡ��
  * @param  void
  * @retval void
  * @attention  ��ͨģʽ�������,�Ҳ�����
  */

/****************��Ƶ����******************/
uint32_t Shoot_Interval = 0;  //λ�û�������,ʵʱ�ɱ�,��ֵԽСλ�û�������Խ��

//����������Ӧ��ģʽ�л�ʱ��ʱ���óɵ�ǰʱ��
uint32_t  Revol_Posit_RespondTime = 0;

/********���**********/
//�����ӵ���,��һ�¼�һ��,��һ�ż�һ��
int16_t Key_ShootNum;//����������

/*����*/
uint8_t Revol_Switch_Left = 0;
u8 	 Revol_Key_Left_Change = 0;

/****************��Ƶ����******************/
#define SHOOT_LEFT_TIME_MAX  150	//��������л����

void SHOOT_NORMAL_Ctrl(void)
{
		static uint32_t shoot_left_time = 0;//�����������ʱ��,ʱ������л�������
	
	/*------ ���̧�����ܴ���һ�� -------*/
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (Revol_Switch_Left == 1)
		{
			Revol_Switch_Left = 2;
		}
		else if (Revol_Switch_Left == 2)
		{
			Revol_Switch_Left = 0;
		}
	}
	else if(!IF_MOUSE_PRESSED_LEFT)		
	{
		Revol_Switch_Left = 1;
		shoot_left_time = 0;//������¼�ʱ
	}
	/*------------------------------------*/
	
	if(IF_MOUSE_PRESSED_LEFT &&	shoot_left_time <= SHOOT_LEFT_TIME_MAX	//�������
			&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;//λ�û���
		shoot_left_time++;//�жϳ���,�л�
		actShoot = SHOOT_SINGLE;	
	}
	else if(IF_MOUSE_PRESSED_LEFT && shoot_left_time > SHOOT_LEFT_TIME_MAX	//��������200ms
				&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		shoot_left_time++;
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_TRIPLE;//����ģʽ
	}
		else if(IF_KEY_PRESSED_B	//����Ƶ������
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_HIGHTF_LOWS;
		shoot_left_time = 0;
	}
	else if(IF_KEY_PRESSED_Z	//����Ƶ��������,�Ƽ�ר��
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_MIDF_HIGHTS;
		shoot_left_time = 0;
	}
		else if(GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//���ģʽ�ҷ��ֶ����ģʽ
	{
		Revolver_mode  = REVOL_POSI_MODE;
		actShoot = SHOOT_BUFF;
		shoot_left_time = 0;
	}
	else
	{
		actShoot = SHOOT_NORMAL;
		Shoot_Interval  = 0;//����������
		Revol_Posit_RespondTime = xTaskGetTickCount();//������Ӧ
		shoot_left_time = 0;
//		Revol_Angle_Clear();//ģʽ�л�ʱ�������
		Key_ShootNum = 0;
	}
	
	if(GIMBAL_IfBuffHit() == FALSE)//�˳��˴��ģʽ
	{
//		First_Into_Buff = TRUE;	
//		Buff_Shoot_Begin = FALSE;
//		buff_fire = FALSE;
	}
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_SINGLE_Ctrl(void)
{
	portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ
	
	CurrentTime = xTaskGetTickCount();
	
	Shoot_Interval = TIME_STAMP_1000MS/8;//���һ��8��
	
	if(RespondTime < CurrentTime
			&& Revol_Switch_Left == 2//�뵯�ֿ���ͬ��
				&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_TRIPLE_Ctrl(void)
{
}

/**
  * @brief  ����Ƶ�����ٿ���
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_HIGHTF_LOWS_Ctrl(void)
{
	static portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ	
	
	CurrentTime = xTaskGetTickCount();
	
	Shoot_Interval = TIME_STAMP_1000MS/20;//ȷ����Ƶ
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}
/**
  * @brief  ����Ƶ�����ٿ���
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_MIDF_HIGHTS_Ctrl(void)
{
	static portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ	
	
	CurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
	
	Shoot_Interval = TIME_STAMP_1000MS/20;//ȷ����Ƶ
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  �����������
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_AUTO_Ctrl(void)
{
	static portTickType CurrentTime     = 0;
	static uint32_t RespondTime_Stop    = 0;//��Ӧ�����ʱ����ֹ
	static uint32_t RespondTime_MobiPre = 0;//��Ӧ�����ʱ���ƶ�Ԥ��
	CurrentTime = xTaskGetTickCount();

/***********************************************************************/
	if( GIMBAL_IfAuto_MobPre_Yaw() == TRUE)	//������Ԥ��			
	{
		Shoot_Interval = TIME_STAMP_1000MS/15;//TIME_STAMP_50MS;//ȷ����Ƶ
		if(GIMBAL_MOBPRE_YAW_FIRE()==TRUE				//�Լ���Ԥ�⵽��λ��
				&& RespondTime_MobiPre < CurrentTime
					&& Key_ShootNum == 0 
						&& IF_MOUSE_PRESSED_LEFT)//�������
		{
			RespondTime_MobiPre = CurrentTime + Shoot_Interval;
			Key_ShootNum ++;
		}
		else//������Ԥ�⵫Ԥ�ⲻ��λ����ֹ��
		{
			Key_ShootNum = 0;
		}
	}
	else if(GIMBAL_IfAuto_MobPre_Yaw() == FALSE)	//û��Ԥ��
	{
		Shoot_Interval = TIME_STAMP_1000MS/5;//ȷ����Ƶ
		if(GIMBAL_MOBPRE_YAW_FIRE()==TRUE		//�Լ���Ԥ�⵽��λ��
				&& RespondTime_Stop < CurrentTime
					&& Key_ShootNum == 0
						&& IF_MOUSE_PRESSED_LEFT)//�������				
		{
			RespondTime_Stop = CurrentTime + Shoot_Interval;//TIME_STAMP_500MS;//ÿ��0.5s������һ��		
//			Key_ShootNum = 3;
			Key_ShootNum ++;
		}
	}
}
/**
  * @brief  ���������ƣ�����ͷ����̨
  * @param  void
  * @retval void
  * @attention  ÿ��500ms�ж�һ�ε�λ����λ����
  */
uint32_t buff_lost_time = 0;//��֡��ʱ������ʱ�����Ϊ��֡
uint32_t buff_change_lost_time = 0;//��װ�׵�֡��ʱ
uint32_t buff_shoot_close = 0;
float buff_stamp = 800;//1100;//��0.8�벹һ��
float buff_lost_stamp = 200;//����200ms��ʧĿ��
float Armor_Change_Delay = 0;
float dy = 80;//1;//130;
void SHOOT_BUFF_Ctrl_Gimbal(void)
{
}







/**
  * @brief  �Ƿ����ֶ����ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfManulHit(void)
{
}


