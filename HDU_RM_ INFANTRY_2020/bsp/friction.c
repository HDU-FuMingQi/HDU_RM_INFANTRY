#include "friction.h"


/*****************高射频必须降低射速，拨弹必须和摩擦轮配合***********************/

//此文件有很多不太合理的地方,只是为了做测试

//此文件用在10ms定时器中断发射中
 
/**************************摩擦轮控制***********************************/


//摩擦轮不同pwm下对应的热量增加值(射速),最好比实际值高5
uint16_t Friction_PWM_HeatInc[5] = {0,  20,  26,  34,  36};//测试时随便定的速度,后面测试更改


//速度等级选择
uint16_t Fric_Speed_Level;

//摩擦轮实际输出速度,用来做斜坡输出
float Friction_Speed_Real;


/************************摩擦轮总控制*****************************/
//摩擦轮速度选择,影响子弹速度,只是测试,系数要后面定
float Friction_PWM_Output[6]     = {0, 460, 505, 583, 695, 685};//关闭  低速  中速  高速  狂暴  哨兵

//速度等级选择
uint16_t Fric_Speed_Level;

//遥控模式下的开启标志位
uint8_t Friction_Switch = 0;//与弹仓开关判断类似

//摩擦轮目标速度
float Friction_Speed_Target;

//摩擦轮等级目标转速
float Frict_Speed_Level_Target;

//摩擦轮实际输出速度,用来做斜坡输出
float Friction_Speed_Real;

//速度选择
#define FRI_OFF  	0
#define FRI_LOW  	1		//Z键射速
#define FRI_MID  	2		//B键射速
#define FRI_HIGH 	3		//左键
#define FRI_MAD  	4		//打符射速
#define FRI_SENTRY  5		//哨兵射速

/**
  * @brief  摩擦轮控制函数
  * @param  void
  * @retval void
  * @attention 摩擦轮停止转动,红外关闭,摩擦轮从关闭到开启,云台抬头归中
  */
void FRICTION_Ctrl()
{
	if(SystemValue==Starting)
	{
		Fric_Speed_Level = FRI_OFF;
		Friction_Speed_Target = 0;
		Friction_Speed_Real   = 0;
	}
	else
	{
		if(ControlMode==REMOTE)
		{
		}
		else if(ControlMode==KEYBOARD)
		{
			FRIC_KeyLevel_Ctrl();
		}
	}
	
	//摩擦轮输出斜坡,注意要先抬头才能进入斜坡
	Friction_Ramp();
	
//	//激光开关
//	if(Friction_Speed_Real > 0 && GIMBAL_IfBuffHit() == FALSE && Magazine_IfOpen() == FALSE)//用实际输出合理一点
//	{
//		Laser_On;
//	}
//	else
//	{
//		Laser_Off;
//		//Laser_On;
//	}
	
	//设置发射速度
	TIM_FrictionPwmOutp(Friction_Speed_Target,Friction_Speed_Target);
}

/**
  * @brief  摩擦轮等级控制,键盘专用,根据等级自动设置射速
  * @param  当前等级
  * @retval void
  * @attention 键盘模式下不关摩擦轮
  */
float debug_fric = 450;//760;//650;
float Fric_Dist_Far = 3;//大于此距离射速拉满
float Fric_Dist_Near = 1.2;//小于此距离射速最低
void FRIC_KeyLevel_Ctrl(void)
{
	static int16_t fric_auto_delay = 0;//防止闪灭频繁换速
	float fric_percent = 1;
	float Fric_Dist_Now = 5;
	
	Fric_Dist_Now = VisionValue.distance/100;\
	
	if (GIMBAL_IfBuffHit( ) == TRUE)
	{
		Fric_Speed_Level = FRI_MAD;//打符模式,最高射速
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}
	else if (IF_MOUSE_PRESSED_LEFT && GIMBAL_IfAutoHit() == FALSE)//非自瞄，正常打击
	{
		Fric_Speed_Level = FRI_HIGH;//一般用高射速
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}
	else if (IF_KEY_PRESSED_Z)//推家射速
	{
		Fric_Speed_Level = FRI_LOW;				
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择		
	}
	else if (IF_KEY_PRESSED_B)
	{
		Fric_Speed_Level = FRI_MID;//高射频低射速模式,推家肉搏用
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}
		
	else if (GIMBAL_IfAutoHit() == TRUE)//自瞄模式
	{
		if(VisionValue.identify_target == TRUE)
		{
			fric_auto_delay = 0;
			
			fric_percent = (Fric_Dist_Now - Fric_Dist_Near) / (Fric_Dist_Far - Fric_Dist_Near);
			fric_percent = constrain_float(fric_percent, 0, 1);
			
			Frict_Speed_Level_Target = Friction_PWM_Output[FRI_LOW] 
										+ (Friction_PWM_Output[FRI_HIGH] - Friction_PWM_Output[FRI_LOW]) * fric_percent;
		}
		else//没识别到，过一段时间后再切换速度
		{
			fric_auto_delay++;
			if(fric_auto_delay >= 20)//连续200ms没识别到
			{
				fric_auto_delay = 200;//防止溢出
				Fric_Speed_Level = FRI_HIGH;//一般用高射速
				Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
			}
		}
		///////自瞄打哨兵
		if(GIMBAL_AUTO_PITCH_SB() == TRUE)//自瞄打哨兵，射速提高
		{
			Fric_Speed_Level = FRI_SENTRY;
			Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
		}
	}
	
	else//防止出错
	{
		Fric_Speed_Level = FRI_HIGH;	
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}

	if( IF_MOUSE_PRESSED_LEFT || Friction_Speed_Target>0 )//不用中间量处理会有BUG——抬头会自动开枪
	{
		//Friction_Speed_Target = debug_fric;
		Friction_Speed_Target = Frict_Speed_Level_Target;
	}
}


/**
  * @brief  当前PWM对应单发子弹热量增加值(射速)
  * @param  void
  * @retval 当前热量递增值
  * @attention 不适用于42mm,可用于客户端数据显示
  */
uint16_t Fric_GetHeatInc(void)
{
	if(GIMBAL_IfAutoHit() == TRUE && !IF_KEY_PRESSED_Z && !IF_KEY_PRESSED_B)
	{
		return JUDGE_usGetSpeedHeat17()*1.4f;
	}
	else
	{
		return Friction_PWM_HeatInc[Fric_Speed_Level];
	}
}

/**
  * @brief  获取当前摩擦轮PWM输出值
  * @param  void
  * @retval 实际PWM值
  * @attention 用来禁止摩擦轮速度过低的情况下拨盘的转动
  */
float Fric_GetSpeedReal(void)
{
	return Friction_Speed_Real;
}

/*************摩擦轮辅助函数****************/

/**
  * @brief  摩擦轮输出斜坡
  * @param  void
  * @retval void
  * @attention 
  */
void Friction_Ramp(void)
{
	if (Friction_Speed_Real < Friction_Speed_Target)//开启
	{
		Friction_Speed_Real += 5;
		if(Friction_Speed_Real > Friction_Speed_Target)
		{
			Friction_Speed_Real = Friction_Speed_Target;
		}
	}
	else if (Friction_Speed_Real > Friction_Speed_Target)//关闭
	{
		Friction_Speed_Real -= 5;
	}
	
	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}
}

/**
  * @brief  摩擦轮输出函数
  * @param  void
  * @retval pwm1  pwm2
  * @attention 左右都是正,以后接线的时候注意三条线的接法
  */
void TIM_FrictionPwmOutp(int16_t pwm1,int16_t pwm2)//8,9
{
	htim2.Instance->CCR1= pwm1+1000;  //持续输出时要大于1ms
	htim2.Instance->CCR2= pwm2+1000;
}













