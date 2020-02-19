#include "shoot_task.h"
int8_t ammunition_mode = stop;
int32_t total_angle_next;
uint8_t shoot_state = unstart;
double initial_speed = 10; //摩擦轮速度
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
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS); //绝对延时
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
     *  下面的部分是键盘模式
			
				
           /**角度值很可能是不对的

 ****************************************************************************************************/

/******拨盘,控制逻辑与云台类似(键盘)*********/

//拨盘电机模式,位置环与速度环
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//拨盘模式选择
//拨盘电机模式,位置环与速度环
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;
eRevolverCtrlMode Revolver_mode;

typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;
eShootAction actShoot;


/*******键盘模式************/

/**
  * @brief  拨盘的键盘模式
  * @param  void
  * @retval void
  * @attention 键盘用位置环控制
  */
void REVOLVER_Key_Ctrl(void)
{
	Revolver_mode = REVOL_POSI_MODE;//防止出错用,默认位置环
	
	SHOOT_NORMAL_Ctrl();//确定射击模式
	
	/*- 确定射击间隔和射击模式 -*/
	switch(actShoot)
	{
		case SHOOT_NORMAL:
			//射击模式选择,默认不打弹
			SHOOT_NORMAL_Ctrl();
		break;
		
		case SHOOT_SINGLE:
			//按一下左键单发,长按连发
			SHOOT_SINGLE_Ctrl();
		break;
		
		case SHOOT_TRIPLE:
			//长按连发
			SHOOT_TRIPLE_Ctrl();
//			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_HIGHTF_LOWS:
			//B高射频
			SHOOT_HIGHTF_LOWS_Ctrl();
		break;
		
		case SHOOT_MIDF_HIGHTS:
			//Z推家
			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_BUFF:
			//打符自动打弹
			Revolver_mode = REVOL_POSI_MODE;
			SHOOT_BUFF_Ctrl_Gimbal();
		break;
		
		case SHOOT_AUTO:
			//右键自瞄时自动打弹
//			SHOOT_AUTO_Ctrl();
		break;		
	}
	
//	/*- 开始发弹,计数减 -*/
//	if(Revolver_mode == REVOL_SPEED_MODE && Fric_GetSpeedReal() > REVOL_CAN_OPEN)
//	{
//		REVOLVER_KeySpeedCtrl();
//	}
//	else if(Revolver_mode == REVOL_POSI_MODE && Fric_GetSpeedReal() > REVOL_CAN_OPEN)
//	{
//		REVOLVER_KeyPosiCtrl();
//	}
}

/************************底盘键盘模式各类模式小函数****************************/
/**
  * @brief  键盘模式下发弹模式选择
  * @param  void
  * @retval void
  * @attention  普通模式清零计算,且不发弹
  */

/****************射频控制******************/
uint32_t Shoot_Interval = 0;  //位置环射击间隔,实时可变,数值越小位置环射击间隔越短

//射击间隔总响应，模式切换时及时重置成当前时间
uint32_t  Revol_Posit_RespondTime = 0;

/********射击**********/
//发射子弹数,按一下加一颗,发一颗减一次
int16_t Key_ShootNum;//鼠标射击计数

/*点射*/
uint8_t Revol_Switch_Left = 0;
u8 	 Revol_Key_Left_Change = 0;

/****************射频控制******************/
#define SHOOT_LEFT_TIME_MAX  150	//左键连按切换间隔

void SHOOT_NORMAL_Ctrl(void)
{
		static uint32_t shoot_left_time = 0;//计算左键连按时间,时间过长切换成连发
	
	/*------ 左键抬起后才能打下一颗 -------*/
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
		shoot_left_time = 0;//左键重新计时
	}
	/*------------------------------------*/
	
	if(IF_MOUSE_PRESSED_LEFT &&	shoot_left_time <= SHOOT_LEFT_TIME_MAX	//左键发弹
			&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;//位置环打弹
		shoot_left_time++;//判断长按,切换
		actShoot = SHOOT_SINGLE;	
	}
	else if(IF_MOUSE_PRESSED_LEFT && shoot_left_time > SHOOT_LEFT_TIME_MAX	//连按大于200ms
				&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		shoot_left_time++;
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_TRIPLE;//连发模式
	}
		else if(IF_KEY_PRESSED_B	//高射频低射速
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_HIGHTF_LOWS;
		shoot_left_time = 0;
	}
	else if(IF_KEY_PRESSED_Z	//高射频极低射速,推家专用
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_MIDF_HIGHTS;
		shoot_left_time = 0;
	}
		else if(GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//打符模式且非手动打符模式
	{
		Revolver_mode  = REVOL_POSI_MODE;
		actShoot = SHOOT_BUFF;
		shoot_left_time = 0;
	}
	else
	{
		actShoot = SHOOT_NORMAL;
		Shoot_Interval  = 0;//重置射击间隔
		Revol_Posit_RespondTime = xTaskGetTickCount();//重置响应
		shoot_left_time = 0;
//		Revol_Angle_Clear();//模式切换时清零计数
		Key_ShootNum = 0;
	}
	
	if(GIMBAL_IfBuffHit() == FALSE)//退出了打符模式
	{
//		First_Into_Buff = TRUE;	
//		Buff_Shoot_Begin = FALSE;
//		buff_fire = FALSE;
	}
}

/**
  * @brief  单发控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_SINGLE_Ctrl(void)
{
	portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时
	
	CurrentTime = xTaskGetTickCount();
	
	Shoot_Interval = TIME_STAMP_1000MS/8;//最快一秒8发
	
	if(RespondTime < CurrentTime
			&& Revol_Switch_Left == 2//与弹仓开关同理
				&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  连发控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_TRIPLE_Ctrl(void)
{
}

/**
  * @brief  高射频低射速控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_HIGHTF_LOWS_Ctrl(void)
{
	static portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时	
	
	CurrentTime = xTaskGetTickCount();
	
	Shoot_Interval = TIME_STAMP_1000MS/20;//确定射频
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}
/**
  * @brief  中射频高射速控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_MIDF_HIGHTS_Ctrl(void)
{
	static portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时	
	
	CurrentTime = xTaskGetTickCount();//当前系统时间
	
	Shoot_Interval = TIME_STAMP_1000MS/20;//确定射频
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  自瞄射击控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_AUTO_Ctrl(void)
{
	static portTickType CurrentTime     = 0;
	static uint32_t RespondTime_Stop    = 0;//响应间隔计时，静止
	static uint32_t RespondTime_MobiPre = 0;//响应间隔计时，移动预测
	CurrentTime = xTaskGetTickCount();

/***********************************************************************/
	if( GIMBAL_IfAuto_MobPre_Yaw() == TRUE)	//开启了预测			
	{
		Shoot_Interval = TIME_STAMP_1000MS/15;//TIME_STAMP_50MS;//确定射频
		if(GIMBAL_MOBPRE_YAW_FIRE()==TRUE				//自己算预测到了位置
				&& RespondTime_MobiPre < CurrentTime
					&& Key_ShootNum == 0 
						&& IF_MOUSE_PRESSED_LEFT)//左键按下
		{
			RespondTime_MobiPre = CurrentTime + Shoot_Interval;
			Key_ShootNum ++;
		}
		else//开启了预测但预测不到位，禁止打弹
		{
			Key_ShootNum = 0;
		}
	}
	else if(GIMBAL_IfAuto_MobPre_Yaw() == FALSE)	//没开预测
	{
		Shoot_Interval = TIME_STAMP_1000MS/5;//确定射频
		if(GIMBAL_MOBPRE_YAW_FIRE()==TRUE		//自己算预测到了位置
				&& RespondTime_Stop < CurrentTime
					&& Key_ShootNum == 0
						&& IF_MOUSE_PRESSED_LEFT)//左键按下				
		{
			RespondTime_Stop = CurrentTime + Shoot_Interval;//TIME_STAMP_500MS;//每隔0.5s三连发一次		
//			Key_ShootNum = 3;
			Key_ShootNum ++;
		}
	}
}
/**
  * @brief  打符射击控制，摄像头在云台
  * @param  void
  * @retval void
  * @attention  每隔500ms判断一次到位，到位开火
  */
uint32_t buff_lost_time = 0;//掉帧延时，超过时间才认为掉帧
uint32_t buff_change_lost_time = 0;//切装甲掉帧延时
uint32_t buff_shoot_close = 0;
float buff_stamp = 800;//1100;//隔0.8秒补一发
float buff_lost_stamp = 200;//连续200ms丢失目标
float Armor_Change_Delay = 0;
float dy = 80;//1;//130;
void SHOOT_BUFF_Ctrl_Gimbal(void)
{
}







/**
  * @brief  是否开启手动打符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfManulHit(void)
{
}


