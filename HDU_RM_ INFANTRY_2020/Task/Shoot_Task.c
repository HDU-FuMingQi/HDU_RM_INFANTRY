#include "shoot_task.h"
int8_t ammunition_mode = stop;
int32_t total_angle_next;
uint8_t shoot_state = unstart;
double initial_speed = 10; //Ħ�����ٶ�
const static fp32 Ammunition_Motor_Position_pid[3] = {0.05, 0, 0};
const static fp32 Ammunition_Motor_Speed_pid[3] = {15, 0.01, 2};

/******����,�����߼�����̨����(����)*********/

//���̵��ģʽ,λ�û����ٶȻ�
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//����ģʽѡ��
//���̵��ģʽ,λ�û����ٶȻ�

eRevolverCtrlMode Revolver_mode  = REVOL_POSI_MODE ;
eShootAction actShoot = SHOOT_NORMAL;

#define    REVOL_CAN_OPEN    350  //Ħ����ʵ���ٶȳ������ֵ��������ת��,����Ħ������СĿ���ٶ����ı�

/************����************/
#define Stuck_Revol_PIDTerm   4000      //PID����������������Ϊ�п��ܿ���
#define Stuck_Speed_Low       60       //�����ٶȵ��������,����Ϊ�п��ܿ���

#define Stuck_SpeedPID_Time   100       //�ٶ����� ms��С,PID����  ms����
#define Stuck_TurnBack_Time   100       //��תʱ��,ʱ��Խ������Խ��
uint32_t Stuck_Speed_Sum = 0;//���㿨������,�ٶȻ�
uint32_t Stuck_Posit_Sum = 0;//���㿨������,λ�û�

portTickType posishoot_time;//�����ʱ����


void ShootFun(void const * argument)
{
	portTickType currentTime;
	Motor_Init2(&Ammunition_Motor, 1, Ammunition_Motor_Position_pid, REVOLVER_PID_POSITION_OUTMAX1, REVOLVER_PID_POSITION_IMAX1,
									Ammunition_Motor_Speed_pid, REVOLVER_PID_SPEED_OUTMAX2, REVOLVER_PID_SPEED_IMAX2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	while (1)
	{
		currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��  
		CAP_Ctrl();//���ݳ�ŵ���ƺ���,ʱ������⣬���Զ�������
		if(SystemValue==Starting)
		{
			REVOLVER_Rest();
			REVOLVER_InitArgument();
		}
		GetEnvironmentShootMode();     //���ó�����������
		
		//��������
		if(Revolver_Heat_Limit() == FALSE //һ��Ҫ�����ٶȡ�λ�ÿ���֮ǰ
		&& GIMBAL_IfBuffHit() != TRUE 
			&& GIMBAL_IfManulHit() != TRUE)//���ʱ��������
		{
			REVOLVER_Rest();//��������,�������������Ҳ��������
		}
		
		switch(ControlMode)
		{
			case REMOTE:
				RemoteShootSpeedSet();
				break;
			case KEYBOARD:
				KeyboardShootSpeedSet();
				break;
		}
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS); //������ʱ
	}
}

/**
  * @brief  ȷ�������״̬
  * @param  void
  * @retval void
  * @attention ͨ��ң����/����
  */
void GetEnvironmentShootMode(void)
{
	switch(ControlMode)
	{
		case REMOTE:   //ң�������ģʽ		
			REVOLVER_Rc_Ctrl();	
			break; 
		case KEYBOARD:  //�������ģʽ
			SHOOT_NORMAL_Ctrl();
			break;
		default :
			break;
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

/**
  * @brief  ���̵�ң���������
  * @param  void
  * @retval void
  * @attention 
  */
void RemoteShootSpeedSet()
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

/**
  * @brief  ���̵ļ����������
  * @param  void
  * @retval void
  * @attention 
  */
void KeyboardShootSpeedSet()
{
		REVOLVER_Key_Ctrl();
	
		if(Revolver_mode == REVOL_SPEED_MODE&&Fric_GetSpeedReal() > REVOL_CAN_OPEN)
		{
			REVOL_SpeedLoop();
		}
		else if(Revolver_mode == REVOL_POSI_MODE&&Fric_GetSpeedReal() > REVOL_CAN_OPEN)
		{
			REVOL_PositionLoop();
		}
		else
		{
			Ammunition_Motor.motor_value->target_speed_rpm = 0;//Ħ���ֹر�,���̲�����
			Revolver_Angle_Rest();//Ħ���ֹرգ��������ڼ�Ĵ�ָ��
		}
}


/*******************���̲���**********************/
//���̲����ٶ�
int16_t Revolver_Speed_Measure;

//���̲����Ƕ�
int16_t Revolver_Angle_Measure;

//����Ŀ��ת��
float  Revolver_Speed_Target;//ת�ٹ������׿���,������ת����6000


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
	
//	SHOOT_NORMAL_Ctrl();//ȷ�����ģʽ
	
	/*- ȷ�������������ģʽ -*/
	switch(actShoot)
	{
//		case SHOOT_NORMAL:
//			//���ģʽѡ��,Ĭ�ϲ���
//			SHOOT_NORMAL_Ctrl();
//		break;
		
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
			SHOOT_AUTO_Ctrl();
		break;		
	}
	
	/*- ��ʼ����,������ -*/
	if(Revolver_mode == REVOL_SPEED_MODE && Fric_GetSpeedReal() > 350)  //������350Ħ����ת����
	{
		REVOLVER_KeySpeedCtrl();
	}
	else if(Revolver_mode == REVOL_POSI_MODE && Fric_GetSpeedReal() > 350)
	{
		REVOLVER_KeyPosiCtrl();
	}
}

/************************���̼���ģʽ����ģʽС����****************************/
/**
  * @brief  ����ģʽ�·���ģʽѡ��
  * @param  void
  * @retval void
  * @attention  ��ͨģʽ�������,�Ҳ�����
  */

/****************��Ƶ����******************/
#define SHOOT_LEFT_TIME_MAX  150	//��������л����

//�����ٶȻ���Ƶ
int16_t Revolver_Freq;

//λ�û�������,ʵʱ�ɱ�,��ֵԽСλ�û�������Խ��
uint32_t Shoot_Interval = 0;  

//����������Ӧ��ģʽ�л�ʱ��ʱ���óɵ�ǰʱ��
uint32_t  Revol_Posit_RespondTime = 0;

/********���**********/
//�����ӵ���,��һ�¼�һ��,��һ�ż�һ��
int16_t Key_ShootNum;//����������

/*����*/
uint16_t Shoot_HeatLimit;//��ǰ�ȼ������������
uint8_t Revol_Switch_Left = 0;
u8 	 Revol_Key_Left_Change = 0;

uint8_t First_Into_Buff = FALSE;
uint8_t Buff_Shoot_Begin = FALSE;
bool 	buff_fire = 0;
bool	buff_change_fire = 0;
/************************************************************************************/

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
	/*************************************************/
	Revolver_mode = REVOL_SPEED_MODE;

	if(JUDGE_usGetShootCold() <= 40)
	{
		Revolver_Freq = 8;//��Ƶѡ��
	}
	else if(JUDGE_usGetShootCold() <= 60 && JUDGE_usGetShootCold() > 40)
	{
		Revolver_Freq = 8;//10;//��Ƶѡ��
	}
	else if(JUDGE_usGetShootCold() <= 80 && JUDGE_usGetShootCold() > 60)
	{
		Revolver_Freq = 8;//12;//��Ƶѡ��
	}
	else if(JUDGE_usGetShootCold() >= 160)//ռ��ﱤ
	{
		Revolver_Freq = 14;//12;//��Ƶѡ��
	}
	else
	{
		Revolver_Freq = 8;//��Ƶѡ��
	}
	
	//�ٶȻ�ת������
	Revolver_Speed_Target = REVOL_SPEED_RATIO/REVOL_SPEED_GRID*Revolver_Freq;
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
	portTickType CurrentTime = 0;
	static uint32_t		RespondTime = 0;//��Ӧ�����ʱ
	static uint32_t lockon_time = 0;//�ȶ���׼һ��ʱ���ɻ���
	
	CurrentTime = xTaskGetTickCount();
	
	if(!IF_MOUSE_PRESSED_RIGH)//�Ҽ��ɿ��Զ���
	{	
		//��֡ͳ�ƣ���̫֡���ز���
		if(VisionValue.identify_buff == FALSE)//ûʶ��Ŀ��
		{
			buff_lost_time++;
			if(buff_lost_time > buff_lost_stamp)
			{
				buff_fire = FALSE;
			}
		}
			else
		{
			buff_lost_time = 0;
			buff_fire = TRUE;//���Կ���
		}
		
		Shoot_Interval = 200;//���������Ƶ,����̫���ֹ����
		
		//ʶ��Ŀ���ҽӽ�Ŀ��
		if( buff_fire == TRUE //�ǳ�ʱ���֡
				&& Vision_If_Armor() == FALSE)//��װ���л�
		{
//			buff_shoot_close = 0;//���¼���δ��׼Ŀ��ʱ��
			Armor_Change_Delay++;
			if(Armor_Change_Delay > 50)
			{
				if(GIMBAL_BUFF_YAW_READY() && GIMBAL_BUFF_PITCH_READY())
				{
					buff_change_lost_time = 0;//ˢ�µ�λ�ж�ʱ��
					buff_change_fire = TRUE;//�������л�����ȶ���ʱ
				}
				else
				{
					buff_change_lost_time++;//�ȶ���λ
					if(buff_change_lost_time > 50)//������֡50ms����Ϊû��λ
					{
						buff_change_fire = FALSE;//�����л��ȶ���ʱ
					}
				}
				
				if(buff_change_fire == TRUE)
				{
					lockon_time++;
				}
				else
				{
					lockon_time = 0;
				}
				
				if( RespondTime < CurrentTime
						&& Key_ShootNum == 0 
							&& lockon_time > dy//80
								&& (VisionValue.vision_yaw_value.value != 0 && VisionValue.vision_pitch_value.value != 0))//�ȶ���λ30ms
				{
					RespondTime = CurrentTime + buff_stamp;//Shoot_Interval;
					Key_ShootNum++;//��һ��
				}
			}
		}
		else//��ʱ���֡�����л���װ��
		{
			lockon_time = 0;
			buff_change_fire = FALSE;//�����л��ȶ���ʱ
			
			if( Vision_If_Armor() == TRUE )//�л�װ�װ�
			{
				Armor_Change_Delay = 0;
				Vision_Clean_Ammor_Flag();//�ȴ��´θ���װ�װ�
			}
			
			RespondTime = CurrentTime-1;//����ˢ�����ʱ��
			Key_ShootNum = 0;
			
//			buff_shoot_close++;
//			lockon_time = 0;
//			if(buff_shoot_close > 100)//����100msû�鵽Ŀ��
//			{
//				RespondTime = CurrentTime-1;//����ˢ�����ʱ��
//				Key_ShootNum = 0;
//			}
		}
	}
	else//��ס�Ҽ��ӹ��Զ���
	{
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
		}
		
		/*------------------------------------*/
		if(IF_MOUSE_PRESSED_LEFT)//�������
		{
			SHOOT_SINGLE_Ctrl();
		}
	}
}


/*********************�ջ�***************************/
/**
  * @brief  ����ģʽ�����ٶȻ�����
  * @param  void
  * @retval void
  * @attention �Ƽ�ģʽ������Ƶ,�����ǵü���һ����־λ��������Ħ���ֵ�������
  */
void REVOLVER_KeySpeedCtrl(void)
{
	REVOL_SpeedStuck();//�����жϼ���ת
}

/**
  * @brief  ����ģʽ����λ�û����� 
  * @param  void
  * @retval void
  * @attention 
  */

//����Ŀ��Ƕ��ۼƺ�,����λ��PID����
float  Revolver_Angle_Target_Sum;
float  Revolver_Buff_Target_Sum;//���ģʽ�µ�Ŀ��ֵ�ݴ棬��ʱĿ��Ƕ���б�´���
float  Revolver_Buff_Ramp = AN_BULLET/40;//40msתһ��,һ�����ܳ���50ms

//�ۼƺ�
float Revolver_Angle_Measure_Sum;//���̲����Ƕ��ۼƺ�,����λ��PID
int16_t Revolver_Angle_Measure_Prev;//�ϴ�ʣ�µ��ۼӺͽǶ�,����Ȧ�������ж�

void REVOLVER_KeyPosiCtrl(void)
{
	static portTickType  CurrentTime = 0;
//	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ
	
	CurrentTime = xTaskGetTickCount();

/*******************���ķ��䲿��***********************/	
	if(Key_ShootNum != 0 && Revol_Posit_RespondTime < CurrentTime)
	{
		Revol_Posit_RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum--;//����������
		Revolver_Buff_Target_Sum += AN_BULLET;//����λ�ü�
		
		posishoot_time = xTaskGetTickCount();//����ָ���´�ʱ��ϵͳʱ��,���ڷ�����ʱ����
	}		
/******************************************************/
	
	if(Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum)//����ת��ȥ
	{
		Revolver_Angle_Target_Sum = RAMP_float(Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Buff_Ramp);
	}
	REVOL_PositStuck();
}






/*********************��Ƶ��������****************************/

/**
  * @brief  ǹ����������
  * @param  void
  * @retval �����Ƿ���
  * @attention  ����Ҫ����һ�²���,����ʣ��ɷ����������ջ�
  *             �����˫ǹ����˺���������
  */
bool Revolver_Heat_Limit(void)
{
	static uint16_t  usShootNumAllow  = 0;
	static uint16_t  usHeatBuffer     = 0;
	static bool_t  IfShootAllow  =  FALSE;

	static  uint16_t  usShootNumBuffer  = 0;
	static  portTickType  ulShootTimeRecord = 0;
	static  uint16_t  usShootHeatRecord = 0;
	static  uint16_t  usShootNumPrev    = 0;
	static  uint16_t  usHeatPrev        = 0;

	static  uint32_t  ulOfflineCnt      = 0;
			uint16_t  usHeatReal		= 0;
			uint16_t  usShootNumReal	= 0;
			uint16_t  usHeatOneShoot	= 30;
			uint16_t  usHeatLimit;
	static  uint32_t  ShootNumBuffer_Error = 0;//������ʱ�������Ϊ��������

	/* ��ȡ���� */
	usHeatReal = JUDGE_usGetRemoteHeat17();
	
	/* ���ߴ��� */
	if (usHeatReal == usHeatPrev)
	{
		
	}
	else
	{
		ulOfflineCnt = 0;
	}
	
	/* ��ȡ������� */
	usShootNumReal  =  JUDGE_usGetShootNum( );
	
	/* ֻҪ���˵������� */
	if (usShootNumReal > usShootNumPrev)
	{
		usShootNumBuffer  += usShootNumReal - usShootNumPrev;
		ulShootTimeRecord  = xTaskGetTickCount( );
		usShootHeatRecord  = usHeatReal;
	}

		/* */
	usHeatOneShoot = Fric_GetHeatInc( );
	if(usHeatOneShoot <= 1)//��ֹ��������ֱ������򵯲��ᶯ
	{
		usHeatOneShoot = 30;
	}
	usHeatLimit    = JUDGE_usGetHeatLimit( );
	if(usHeatLimit <= 30)
	{
		usHeatLimit = 240;//��ֹ���ݳ���
	}

	/* ʣ������ */
	if (usHeatReal <= usHeatLimit)
	{
		usHeatBuffer = usHeatLimit - usHeatReal;
	}
	else
	{
		usHeatBuffer = 0;
	}

	if (usHeatBuffer > usHeatOneShoot)//ʣ���������ڴ�һ����������
	{
		/* ���ܴ�����ӵ���Ŀ */
		usShootNumAllow = (uint16_t)(usHeatBuffer / usHeatOneShoot);// - 1;
	}
	else
	{
		usShootNumAllow = 0;//ʣ���������ͣ�������
	}

		/**/
	if ( abs(Revolver_Speed_Measure) <= 100	
			&& xTaskGetTickCount( ) - ulShootTimeRecord > TIME_STAMP_30MS/*TIME_STAMP_100MS*/ 
				&& (usHeatReal > usShootHeatRecord + 3 || usHeatReal == 0) )
	{
		/* ���� */
		usShootNumBuffer = 0;
	}

	if (usShootNumAllow <= 1)
	{
		IfShootAllow = FALSE;
	}
	else if (usShootNumBuffer < usShootNumAllow)
	{
		IfShootAllow = TRUE;
	}
	else
	{
		IfShootAllow = FALSE;
	}
	
	/* ��¼��ʱ��������� */  
	usShootNumPrev  =  usShootNumReal;
	usHeatPrev 		=  usHeatReal;
	
		if(usShootNumBuffer > 6)//��ֹ��������ʱ�����������
	{
		ShootNumBuffer_Error++;
	}
	else
	{
		ShootNumBuffer_Error = 0;
	}
	
	if(ShootNumBuffer_Error > TIME_STAMP_1000MS*5)//����5�����6��
	{
		usShootNumBuffer = 0;
		ShootNumBuffer_Error = 0;
	}
	
	
	if (ulOfflineCnt < 100)
	{
		return  IfShootAllow;
	}
	else
	{
		return  TRUE;	
	}


}

/********************************************************************************/

/**
  * @brief  ����ʧ�ر���
  * @param  void
  * @retval void
  * @attention ���������0
  */
void REVOLVER_StopMotor(void)
{
	Ammunition_Motor.Motor_PID_Speed.out = 0;
}

/**
  * @brief  ���̲�����ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_InitArgument(void)
{
	/* Ŀ��ֵ */
	Revolver_Speed_Target = 0;
	
	/* ��� */
	Key_ShootNum    = 0;
	Shoot_HeatLimit = 240;//������ʼ��
	Revolver_Freq   = 0;//��Ƶ��ʼ��
	
	/* λ�û�Ŀ��Ƕ� */
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure;//������0,�����ϵ�ᷴת
	Revolver_Buff_Target_Sum  = Revolver_Angle_Measure;
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention ǹ�ڳ���������
  */
void REVOLVER_Rest(void)
{
	Key_ShootNum = 0;//λ�û���������
	Revolver_Speed_Target = 0;//�ٶȻ�ֹͣת��
	
	//�ٶȻ�λ������
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//�ϴ�λ������
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Measure;
	
	//PID��������
	Ammunition_Motor.Motor_PID_Position.out = 0;
	Ammunition_Motor.Motor_PID_Position.Iout = 0;
	
}

/**
  * @brief  ���̽Ƕ�����
  * @param  void
  * @retval void
  * @attention ģʽ�л�ʱ��,��ֹ�´��л�ȥ��ͻȻ��һ��
  */
void Revolver_Angle_Rest(void)
{
	Key_ShootNum = 0;
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//�ϴ�λ������
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Target_Sum;
}

/*******************���̵�����ݸ���*********************/

/**
  * @brief  ��ȡ����Ƕ�
  * @param  CAN����
  * @retval void
  * @attention  CAN2�ж��е���
  */
void REVOLVER_UpdateMotorAngle()
{
	 Revolver_Angle_Measure = Ammunition_Motor.motor_value->angle;
}

/**
  * @brief  ��ȡ���ת��
  * @param  CAN����
  * @retval void
  * @attention  CAN2�ж��е���
  */
void REVOLVER_UpdateMotorSpeed(int16_t speed)
{
}

/**
  * @brief  ͳ��ת���Ƕ��ܺ�
  * @param  void
  * @retval void
  * @attention �л���ģʽ֮��ǵ����� 
  */
void REVOL_UpdateMotorAngleSum(void)
{
	//�ٽ�ֵ�жϷ�
	if (abs(Revolver_Angle_Measure - Revolver_Angle_Measure_Prev) > 4095)//ת����Ȧ
	{		
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (Revolver_Angle_Measure < Revolver_Angle_Measure_Prev)//����Ȧ�ҹ����
		{
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			Revolver_Angle_Measure_Sum += 8191 - Revolver_Angle_Measure_Prev + Revolver_Angle_Measure;
		}
		else
		{
			//������һȦ
			Revolver_Angle_Measure_Sum -= 8191 - Revolver_Angle_Measure + Revolver_Angle_Measure_Prev;
		}
	}
	else      
	{
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
		Revolver_Angle_Measure_Sum += Revolver_Angle_Measure - Revolver_Angle_Measure_Prev;
	}

	//��¼��ʱ����Ƕ�,��һ�μ���ת���ǶȲ���,�����ж��Ƿ�ת��1Ȧ
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;
}


/***********************PID����**********************/

/**
  * @brief  �ٶȻ�PID���������    
  * @param  void
  * @retval void
  * @attention  ң��ֻ���ٶȻ�     ֻ����ң����
  */
void REVOL_SpeedLoop(void)
{
	PID_Calc(&Ammunition_Motor.Motor_PID_Speed, Ammunition_Motor.motor_value->speed_rpm,
		 Ammunition_Motor.motor_value->target_speed_rpm);
	set_moto1234_current(&hcan2, Ammunition_Motor.Motor_PID_Speed.out, 0, 0, 0);
}

/**
  * @brief  λ�û�PID���������
  * @param  void
  * @retval void
  * @attention  ����ģʽ
  */

void REVOL_PositionLoop(void)
{
	//��ȡת�����ܽǶ�ֵ
	REVOL_UpdateMotorAngleSum( );
	
	PID_Calc(&Ammunition_Motor.Motor_PID_Speed, Ammunition_Motor.motor_value->speed_rpm,
		 Ammunition_Motor.motor_value->target_speed_rpm);
	
	Ammunition_Motor.motor_value->target_speed_rpm = Revolver_Angle_Target_Sum;
	
	set_moto1234_current(&hcan2, Ammunition_Motor.Motor_PID_Speed.out, 0, 0, 0);
	
	
}

/*****************************��������**************************************/

/**
  * @brief  �ٶȻ�ʽ��������
  * @param  void
  * @retval void
  * @attention  ��ס�ͷ�תn��
  */
void REVOL_SpeedStuck(void)
{
	static uint16_t  stuck_time    = 0;//������ʱ
	static uint16_t  turnback_time = 0;//��ת��ʱ
	static bool Revol_Speed_ifStuck = FALSE;//�����ж�
	
	if (Revol_Speed_ifStuck == TRUE)//��ȷ�Ͽ���,��ʼ��ת��ʱ
	{
		Ammunition_Motor.motor_value->target_speed_rpm = -4000;//��ת
		turnback_time++;//��תһ��ʱ��

		if (turnback_time > Stuck_TurnBack_Time)//��ת���
		{
			turnback_time  = 0;
			Revol_Speed_ifStuck = FALSE;//������ת
		}			
	}
	else
	{
		if ( abs(Ammunition_Motor.Motor_PID_Speed.out) >= Stuck_Revol_PIDTerm //PID�������
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//�ٶȹ���
		{
			stuck_time++;//������ʱ
		}
		else
		{
			stuck_time = 0;//û�г�ʱ�俨��,��ʱ����
		}

		if (stuck_time > Stuck_SpeedPID_Time)//���˳���60ms
		{
			Stuck_Speed_Sum++;//��������,����е��Ĵ��ֵܵ�����
			stuck_time = 0;
			Revol_Speed_ifStuck = TRUE;//��ǿ��Խ��뵹ת��ʱ
		}
	}
}



/**
  * @brief  λ�û�ʽ��������
  * @param  void
  * @retval void
  * @attention  ��ס�ͷ�תn��
  */

void REVOL_PositStuck(void)
{
	static uint16_t  stuck_time      = 0;//������ʱ
	static uint16_t  turnback_time   = 0;//��ת��ʱ
	static bool Revol_Posit_ifStuck = FALSE;//�����ж�
	
	if (Revol_Posit_ifStuck == TRUE)//������ʼ��ת��ʱ
	{
		Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum;//������ת,��ת�ر�����Ҫ��ת����λ��
		Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
		Revol_Posit_ifStuck = FALSE;//��Ϊ��ʱ���ٿ�����
		turnback_time = 0;//��תʱ������,Ϊ�´ε�ת��׼��	
	}
	else
	{
		if ( abs(Ammunition_Motor.Motor_PID_Speed.out)  >= Stuck_Revol_PIDTerm //PID�������
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//�ٶȹ���
		{
			stuck_time++;//ͳ�ƿ��˶೤ʱ��
		}
		else
		{
			stuck_time = 0;//������,ʱ������
		}
		
		if (stuck_time > Stuck_SpeedPID_Time)//��̫����,��ʾҪ��ת
		{
			//��ת���ܷ���Revol_Posit_ifStuck == TRUE��,����Ͳ��Ƕ�һ�ε�ת1/2����
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum - AN_BULLET ;//��ת 1��  
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			
			Stuck_Posit_Sum++;//��������,����е��Ĵ��ֵܵ�����
			stuck_time = 0;
			Revol_Posit_ifStuck = TRUE;//������ǵ�ת��ʱ����	
		}
	}
}

/**
  * @brief  ����ʱ���ȡ
  * @param  void
  * @retval λ�û�ʵʱָ��ʱ��
  * @attention  ���ڷ�����ʱ����
  */
portTickType REVOL_uiGetRevolTime(void)
{
	return posishoot_time;
}











