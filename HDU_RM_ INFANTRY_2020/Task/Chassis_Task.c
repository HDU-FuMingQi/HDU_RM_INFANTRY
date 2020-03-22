/*********************************/

//本文件中对各个模式的最大输出量均保留了深大代码的
//HDU的写在mysystem.h里

//而且小陀螺和底盘跟随云台目前好像没有键可用

/********************************/
#include "chassis_task.h"
#include "judge.h"

eChassisAction actChassis=CHASSIS_NORMAL;   //默认底盘不跟随云台行走
eChassisCtrlMode  modeChassis=CHASSIS_GYRO_MODE; //默认为陀螺仪模式行走

extKalman_t Chassis_Error_Kalman;//定义一个kalman指针
PidTypeDef Chassis_Follow_PID;
Chassis_Speed absolute_chassis_speed;
int16_t chassis_motor[4];		//四个轮子目标转速

const static fp32 nothing[3]={0,0,0};
const static fp32 motorid1_speed_pid[3] ={22,0,12};
const static fp32 motorid2_speed_pid[3] ={22,0, 12};
const static fp32 motorid3_speed_pid[3] ={22,0, 12};
const static fp32 motorid4_speed_pid[3] ={22,0, 12};
const static fp32 Chassis_Follow_pid[3]={0.00001,0,0.00012};


/**
  * @brief  底盘参数初始化
  * @param  void
  * @retval void
  * @attention 
  */
//底盘电流限幅
#define iTermChassis_Max             3000     //微分限幅

	#define		Omni_Speed_Max            9000//7600     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
	#define		STANDARD_MAX_NORMAL       9000//7600     //平地开车最快速度，防止摇杆比例*660超过这个值
	#define		REVOLVE_MAX_NORMAL        9000//7600     //平地扭头最快速度
	#define     REVOLVE_KD                (235.f)
	#define     REVOLVE_ANGLE             35

//陀螺仪模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Gyro_Chassis_Standard, kKey_Gyro_Chassis_Revolve;//平移，旋转

//机械模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Mech_Chassis_Standard, kKey_Mech_Chassis_Revolve;//平移，旋转

/**************限幅**************/
#define   LIMIT_CHASSIS_MAX         9000     //功率限制情况下底盘单个电机最大输出

float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//限制PID运算最终输出值,底盘功率限制,根据电机转速实时变化
float Chassis_Standard_Move_Max;//底盘前后左右平移限速
float Chassis_Revolve_Move_Max;//底盘左右旋转限速,根据不同运动模式实时更改,所以不能用宏定义

float Chassis_Limit_Output_Max=LIMIT_CHASSIS_MAX;//底盘功率限幅

void CHASSIS_InitArgument(void)
{
	kKey_Mech_Chassis_Revolve = 40;//键盘机械模式下扭头速度响应快慢,别太大,不然扭头太快
	kKey_Gyro_Chassis_Revolve = -10;//-8.1;//注意正负,键盘陀螺仪模式下底盘跟随云台转动的速度,别给太大,否则震荡会很严重
	
	Chassis_Standard_Move_Max  = Omni_Speed_Max;//6800;//9000;//摇杆水平移动限幅
	Chassis_Revolve_Move_Max   = Omni_Speed_Max;//6800;//7000;//摇杆左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动
	
	KalmanCreate(&Chassis_Error_Kalman, 1, 0);      //将在闭环里使用，现在还不太清楚

}


void ChassisFun(void const * argument)
{
	portTickType currentTime;
	Chassis_Follow_PID.angle_max=8192;
	Chassis_Follow_PID.angle_min=0;
	Chassis_Follow_PID.Dead_Zone=100;
	Chassis_Follow_PID.I_Separation=1e30;
	Chassis_Follow_PID.gama=0.2;
	PID_Init(&Chassis_Follow_PID,PID_POSITION,Chassis_Follow_pid,0.015,0.003);//底盘跟随云台PID

	
	/******************底盘电机PID*****************************************/
	Motor_Init(&Chassis_Motor1,1,nothing,0,0,motorid1_speed_pid,20000,10000);//
	Motor_Init(&Chassis_Motor2,2,nothing,0,0,motorid2_speed_pid,20000,10000);
	Motor_Init(&Chassis_Motor3,3,nothing,0,0,motorid3_speed_pid,20000,10000);
	Motor_Init(&Chassis_Motor4,4,nothing,0,0,motorid4_speed_pid,20000,10000);
	while(1)
	{
		currentTime = xTaskGetTickCount();//当前系统时间
		GetEnvironmentChassisMode();	//设置车辆所处环境
		LimitChassisMotorCurrent();		//根据所处环境进行限流
		/***********************选择控制模式为遥控器/键盘进行控制***************/
		switch(ControlMode)
		{
			case REMOTE:
				RemoteControlChassis();
				break;
			case KEYBOARD:
				KeyboardControlChassis();
				break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
	}
}
/**
  * @brief  设置底盘电机输出电流最大值
  * @param  电机1最大值，电机2最大值，电机3最大值，电机4最大值
  * @retval void
  * @attention 
  */
void SetChassisMotorMaxCurrent(const int16_t max1,const int16_t max2,const int16_t max3,const int16_t max4)
{
	Chassis_Motor1.Motor_PID_Speed.max_out=max1;
	Chassis_Motor2.Motor_PID_Speed.max_out=max2;
	Chassis_Motor3.Motor_PID_Speed.max_out=max3;
	Chassis_Motor4.Motor_PID_Speed.max_out=max4;
}
/**
  * @brief  根据不同情况进行限流
  * @param  void
  * @retval void
  * @attention 
  */
void LimitChassisMotorCurrent(void)
{
	switch(actChassis)
	{
		case CHASSIS_NORMAL:		//底盘不跟随云台
			switch(EnvironmentMode)
			{
				case NOMAL:			//普通地形
					SetChassisMotorMaxCurrent(NOMOAL_CHASSIS_MAX1,NOMOAL_CHASSIS_MAX2,NOMOAL_CHASSIS_MAX3,NOMOAL_CHASSIS_MAX4);
					break;
				case CLIMBING:		//爬坡地形
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_MAX1,CLIMBING_CHASSIS_MAX2,CLIMBING_CHASSIS_MAX3,CLIMBING_CHASSIS_MAX4);
					break;
			}
			break;

		case CHASSIS_PISA:				//45°模式，与底盘跟随云台限流相同
			switch(EnvironmentMode)
			{
				case NOMAL:			//普通地形
					SetChassisMotorMaxCurrent(NOMAL_FOLLOW_CHASSIS_MAX1,NOMAL_FOLLOW_CHASSIS_MAX2,NOMAL_FOLLOW_CHASSIS_MAX3,NOMAL_FOLLOW_CHASSIS_MAX4);
					break;
				case CLIMBING:		//爬坡地形
					SetChassisMotorMaxCurrent(CLIMBING_FOLLOW_CHASSIS_MAX1,CLIMBING_FOLLOW_CHASSIS_MAX2,CLIMBING_FOLLOW_CHASSIS_MAX3,CLIMBING_FOLLOW_CHASSIS_MAX4);
					break;
			}
			break;

		case CHASSIS_CORGI:				//扭屁股模式，跟小陀螺限流相同
			switch(EnvironmentMode)
			{
				case NOMAL:			//普通地形
						SetChassisMotorMaxCurrent(NOMAL_GYRO_CHASSIS_MAX1,NOMAL_GYRO_CHASSIS_MAX2,NOMAL_GYRO_CHASSIS_MAX3,NOMAL_GYRO_CHASSIS_MAX4);
						break;
				case CLIMBING:		//爬坡地形
					SetChassisMotorMaxCurrent(CLIMBING_GYRO_CHASSIS_MAX1,CLIMBING_GYRO_CHASSIS_MAX2,CLIMBING_GYRO_CHASSIS_MAX3,CLIMBING_GYRO_CHASSIS_MAX4);
					break;
			}
			break;
			
		case  CHASSIS_SZUPUP:  //手动爬坡模式
			SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_SZUPUP_MAX1,CLIMBING_CHASSIS_SZUPUP_MAX2,CLIMBING_CHASSIS_SZUPUP_MAX3,CLIMBING_CHASSIS_SZUPUP_MAX4);
			break;
		
		case  CHASSIS_MISS:    //自动闪避模式
			switch(EnvironmentMode)
			{
				case NOMAL:			//普通地形
						SetChassisMotorMaxCurrent(NOMAL_CHASSIS_MISS_MAX1,NOMAL_CHASSIS_MISS_MAX2,NOMAL_CHASSIS_MISS_MAX3,NOMAL_CHASSIS_MISS_MAX4);
						break;
				case CLIMBING:		//爬坡地形
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_MISS_MAX1,CLIMBING_CHASSIS_MISS_MAX2,CLIMBING_CHASSIS_MISS_MAX3,CLIMBING_CHASSIS_MISS_MAX4);
					break;
			}
			break;
			
		case CHASSIS_GYROSCOPE:			//小陀螺模式
			switch(EnvironmentMode)
			{
				case NOMAL:			//普通地形
						SetChassisMotorMaxCurrent(NOMAL_CHASSIS_GYROSCOPE_MAX1,NOMAL_CHASSIS_GYROSCOPE_MAX2,NOMAL_CHASSIS_GYROSCOPE_MAX3,NOMAL_CHASSIS_GYROSCOPE_MAX4);
						break;
				case CLIMBING:		//爬坡地形
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_GYROSCOPE_MAX1,CLIMBING_CHASSIS_GYROSCOPE_MAX2,CLIMBING_CHASSIS_GYROSCOPE_MAX3,CLIMBING_CHASSIS_GYROSCOPE_MAX4);
					break;
			}
			break;
			
		case CHASSIS_FOLLOW_GIMBAL:			//底盘跟随云台
			switch(EnvironmentMode)
			{
				case NOMAL:			//普通地形
						SetChassisMotorMaxCurrent(NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX1,NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX2,NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX3,NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX4);
						break;
				case CLIMBING:		//爬坡地形
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX1,CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX2,CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX3,CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX4);
					break;
			}
			break;
		
		default:
			break;
			
	}
}
/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention 
  */



void RemoteControlChassis(void)
{
	/***********************************确定底盘四个电机的目标速度*****************************************/
	switch(actChassis)
	{
		case CHASSIS_FOLLOW_GIMBAL://跟随云台
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY;
			absolute_chassis_speed.vw=PID_Calc(&Chassis_Follow_PID,FindMinAnglePNY(),0);//PID使底盘跟随云台速度
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//计算各个电机的目标速度
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//设置各个电机的目标速度
			break;
		case CHASSIS_NORMAL://不跟随云台
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY;
			absolute_chassis_speed.vw=0;
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//计算各个电机的目标速度
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//设置各个电机的目标速度
			break;
		case CHASSIS_PISA:		//45°模式
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY;
			absolute_chassis_speed.vw=PID_Calc(&Chassis_Follow_PID,FindMinAngleFortyFive(),0);//PID使底盘跟随云台速度
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//计算各个电机的目标速度
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//设置各个电机的目标速度
			break;
		case CHASSIS_GYROSCOPE:		//小陀螺模式
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY;
			absolute_chassis_speed.vw=0.006;
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//计算各个电机的目标速度
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//设置各个电机的目标速度
			break;
		case CHASSIS_CORGI:		//扭屁股模式
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY;
			absolute_chassis_speed.vw=PID_Calc(&Chassis_Follow_PID,FindMinAngleFortyFive(),0);//PID使底盘跟随云台速度
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//计算各个电机的目标速度
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//设置各个电机的目标速度
			break;
		default:
			break;
	}
	/************************************底盘电机速度环计算*********************************************/
			PID_Calc(&Chassis_Motor1.Motor_PID_Speed,Chassis_Motor1.motor_value->speed_rpm,
				Chassis_Motor1.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor2.Motor_PID_Speed,Chassis_Motor2.motor_value->speed_rpm,
							Chassis_Motor2.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor3.Motor_PID_Speed,Chassis_Motor3.motor_value->speed_rpm,
							Chassis_Motor3.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor4.Motor_PID_Speed,Chassis_Motor4.motor_value->speed_rpm,
							Chassis_Motor4.motor_value->target_speed_rpm);
	/************************************将电流参数发送给电机*********************************************/
		Chassis_Power_Limit();//功率限制,电流重新分配
		set_moto1234_current(&hcan1,Chassis_Motor1.Motor_PID_Speed.out,Chassis_Motor2.Motor_PID_Speed.out
									,Chassis_Motor3.Motor_PID_Speed.out,Chassis_Motor4.Motor_PID_Speed.out);
}

/*****************底盘功率*************************/

#define  CHAS_CURRENT_LIMIT        40000    //四个轮子的速度总和最大值,单个输出*4,限功率调比例可用

float fChasCurrentLimit = CHAS_CURRENT_LIMIT;//限制4个轮子的速度总和
float fTotalCurrentLimit;//电流分配,平地模式下分配是均匀的
float WARNING_REMAIN_POWER = 60;//裁判系统剩余焦耳能量低于这个数值则开始限功率,40扭屁股会超功率,平地开不会超

/**
  * @brief  底盘功率限制
  * @param  void
  * @retval void
  * @attention  在底盘输出计算后调用,主要是比例的算法,ICRA
  */
void Chassis_Power_Limit(void)
{
	/*********************祖传算法*************************/
	float    kLimit = 0;//功率限制系数
	float    chassis_totaloutput = 0;//统计总输出电流
	float    Joule_Residue = 0;//剩余焦耳缓冲能量
	int16_t  judgDataCorrect = 0;//裁判系统数据是否可用	
	static int32_t judgDataError_Time = 0;
	
	
	judgDataCorrect = JUDGE_sGetDataState();//裁判系统数据是否可用
	Joule_Residue = JUDGE_fGetRemainEnergy();//剩余焦耳能量	
	
	//统计底盘总输出
	chassis_totaloutput=abs_float(Chassis_Motor1.Motor_PID_Speed.out)+abs_float(Chassis_Motor2.Motor_PID_Speed.out)
														+abs_float(Chassis_Motor3.Motor_PID_Speed.out)+abs_float(Chassis_Motor4.Motor_PID_Speed.out);
	if(judgDataCorrect == JUDGE_DATA_ERROR)//裁判系统无效时强制限速
	{
		judgDataError_Time++;
		if(judgDataError_Time > 100)
		{
			fTotalCurrentLimit = 10000;//降为最大的1/4
		}
	}
	else
	{
		judgDataError_Time = 0;
		//剩余焦耳量过小,开始限制输出,限制系数为平方关系
		if(Joule_Residue < WARNING_REMAIN_POWER)
		{
			kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
						* (float)(Joule_Residue / WARNING_REMAIN_POWER);
			
			fTotalCurrentLimit = kLimit * fChasCurrentLimit;
		}
		else   //焦耳能量恢复到一定数值
		{
			fTotalCurrentLimit = fChasCurrentLimit;
		}
	}
	
	//底盘各电机电流重新分配
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);	
	}
}

/**
  * @brief  键盘控制方式
  * @param  void
  * @retval void
  * @attention 
  */

#define Omni_SupCap_Max				 10000		//电容放电情况下最大速度
#define 	Omni_Speed_Max            9000     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值


void KeyboardControlChassis(void)
{
	float speed_max;
	if(Cap_Out_Can_Open() == TRUE)//电容放电
	{
		speed_max = Omni_SupCap_Max;
	}
	else
	{
		speed_max = Omni_Speed_Max;
	}
	
	CHAS_Key_Ctrl();
	Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//计算各个电机的目标速度
	Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//设置各个电机的目标速度
	
	/************************************底盘电机速度环计算*********************************************/
			PID_Calc(&Chassis_Motor1.Motor_PID_Speed,Chassis_Motor1.motor_value->speed_rpm,
				Chassis_Motor1.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor2.Motor_PID_Speed,Chassis_Motor2.motor_value->speed_rpm,
							Chassis_Motor2.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor3.Motor_PID_Speed,Chassis_Motor3.motor_value->speed_rpm,
							Chassis_Motor3.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor4.Motor_PID_Speed,Chassis_Motor4.motor_value->speed_rpm,
							Chassis_Motor4.motor_value->target_speed_rpm);
	/************************************将电流参数发送给电机*********************************************/
		Chassis_Motor1.Motor_PID_Speed.out = constrain_float(Chassis_Motor1.Motor_PID_Speed.out ,-speed_max, speed_max);
		Chassis_Motor2.Motor_PID_Speed.out = constrain_float(Chassis_Motor2.Motor_PID_Speed.out ,-speed_max, speed_max);
		Chassis_Motor3.Motor_PID_Speed.out = constrain_float(Chassis_Motor3.Motor_PID_Speed.out ,-speed_max, speed_max);
		Chassis_Motor4.Motor_PID_Speed.out = constrain_float(Chassis_Motor4.Motor_PID_Speed.out ,-speed_max, speed_max);
		if(actChassis == CHASSIS_SZUPUP)//爬坡前轮会打滑，所以限得比后轮要小,,此处怎么限制实验再说
		{
			Chassis_Motor1.Motor_PID_Speed.out = constrain_float(Chassis_Motor1.Motor_PID_Speed.out ,-speed_max, speed_max);
			Chassis_Motor2.Motor_PID_Speed.out = constrain_float(Chassis_Motor2.Motor_PID_Speed.out ,-speed_max, speed_max);
			Chassis_Motor3.Motor_PID_Speed.out = constrain_float(Chassis_Motor3.Motor_PID_Speed.out ,-Omni_Speed_Max, Omni_Speed_Max);
			Chassis_Motor4.Motor_PID_Speed.out = constrain_float(Chassis_Motor4.Motor_PID_Speed.out ,-Omni_Speed_Max, Omni_Speed_Max);
		}
		Chassis_Power_Limit();//功率限制,电流重新分配
		set_moto1234_current(&hcan1,Chassis_Motor1.Motor_PID_Speed.out,Chassis_Motor2.Motor_PID_Speed.out
									,Chassis_Motor3.Motor_PID_Speed.out,Chassis_Motor4.Motor_PID_Speed.out);
}
/**
  * @brief  确定地形和底盘状态
  * @param  void
  * @retval void
  * @attention 通过遥控器/键盘
  */
void GetEnvironmentChassisMode(void)
{
	switch(ControlMode)
	{
		case REMOTE:
			//选择车底盘模式
			if(rc.sw1==1)
				actChassis=CHASSIS_NORMAL;	//底盘不跟随云台

			else if(rc.sw1==3)
				actChassis=CHASSIS_FOLLOW_GIMBAL;	//底盘跟随云台
			else if(rc.sw1==2)	
				actChassis=CHASSIS_GYROSCOPE;		//小陀螺模式
			//选择所处环境
			break;
	
		case KEYBOARD:    //键盘模式
			/*------------------普通模式,进行模式切换判断-------------------*/	
			Chassis_NORMAL_Mode_Ctrl();								
			break;
		
		default:
			break;
	}
}

/**
  * @brief  麦轮解算
  * @param  
  * @retval 
  * @attention 转化为速度Motor[i].target_speed_rpm
  */
void mecanum_calc(Chassis_Speed *speed, int16_t* out_speed)
{
	  int16_t wheel_rpm[4];
	  float wheel_rpm_ratio;

	  wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*3.14f)*CHASSIS_DECELE_RATIO*1000;

	  wheel_rpm[0] = ( speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio;
	  wheel_rpm[1] = ( speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio;
	  wheel_rpm[2] = (-speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio;
	  wheel_rpm[3] = (-speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B)) * wheel_rpm_ratio;

	  memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));
}
void Mecanum_Set_Motor_Speed(int16_t*out_speed ,moto_measure_t* Motor )
{
	Motor[0].target_speed_rpm=out_speed[0];
	Motor[1].target_speed_rpm=out_speed[1];
	Motor[2].target_speed_rpm=out_speed[2];
	Motor[3].target_speed_rpm=out_speed[3];
	
}

/*
 * @param absolute_speed 绝对坐标需要的速度
 * @param angle 云台相对于底盘的角度
 */

void Absolute_Cal(Chassis_Speed* absolute_speed , float angle )	
{
	float angle_hd=angle* PI / 180;
	Chassis_Speed temp_speed;
	temp_speed.vw=absolute_speed->vw;
	temp_speed.vx=absolute_speed->vx*cos(angle_hd)-absolute_speed->vy*sin(angle_hd);
	temp_speed.vy=absolute_speed->vx*sin(angle_hd)+absolute_speed->vy*cos(angle_hd);
	mecanum_calc(&temp_speed,chassis_motor);
}

/**
  * @brief  找出与+-y轴最小偏差角
  * @param  void
  * @retval 偏差角，角度制
  * @attention 通过遥控器/键盘
  */
float FindMinAnglePNY(void)
{
	float temp1=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1;
	float temp2=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE2;
	if(temp1>4096)
		temp1-=8192;
	else if(temp1<-4096)
		temp1+=8192;
	if(temp2>4096)
		temp2-=8192;
	else if(temp2<-4096)
		temp2+=8192;
	return (abs(temp1)<abs(temp2)?temp1:temp2);
}
/**
  * @brief  找出与45°轴最小偏差角
  * @param  void
  * @retval 偏差角，角度制
  * @attention 通过遥控器/键盘
  */
float FindMinAngleFortyFive(void)
{
	float temp1=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE1;
	float temp2=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE2;
	float temp3=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE3;
	float temp4=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE4;
	float mintemp1,mintemp2;
	if(temp1>4096)
		temp1-=8192;
	else if(temp1<-4096)
		temp1+=8192;
	if(temp2>4096)
		temp2-=8192;
	else if(temp2<-4096)
		temp2+=8192;
	if(temp3>4096)
		temp3-=8192;
	else if(temp3<-4096)
		temp3+=8192;
	if(temp4>4096)
		temp4-=8192;
	else if(temp4<-4096)
		temp4+=8192;
	mintemp1=(abs(temp1)<abs(temp2)?temp1:temp2);
	mintemp2=(abs(temp3)<abs(temp4)?temp3:temp4);
	return (abs(mintemp1)<abs(mintemp2)?mintemp1:mintemp2);
}



/**
  * @brief  底盘启动状态
  * @param  void
  * @retval void
  * @attention 状态值0
  */

float Slope_Chassis_Move_Z;//斜坡计算出的移动变量,这是目标输出量的实时斜坡值

void CHASSIS_REST(void)
{
	Slope_Chassis_Move_Z = 0;//扭屁股实时输出斜坡
	absolute_chassis_speed.vx  = 0;
	absolute_chassis_speed.vy  = 0;
	absolute_chassis_speed.vw  = 0;
}



/***********************************************************************************************
     *  下面的部分是键盘模式
			
				


 ****************************************************************************************************/

/***********底盘各类模式的一些辅助定义*************/
/**
  * @brief  键盘控制底盘移动
  * @param  void
  * @retval void
  * @attention 模式选择,进入某模式后记得写退出到普通模式的判断
  * 无按键按下会一直处于自动闪避模式,除了模式切换外的按键按下则处于模式切换选择模式
  */

//每2ms执行一次任务函数，绝对延时
uint8_t remot_change = TRUE;

//扭屁股
bool Chass_Switch_F = 1;
u8 	 Chass_Key_F_Change = 0;

//45°
bool Chass_Switch_X = 1;
u8 	 Chass_Key_X_Change = 0;

//小陀螺
bool Chass_Switch_G = 1;
u8 	 Chass_Key_G_Change = 0;

//不同模式下,斜坡函数对应的时间值,一般用普通速度就行
#define    TIME_INC_NORMAL           10//6//4	  //键盘斜坡,越大增加速度越快,完成时间越短
#define    TIME_DEC_NORMAL           500//180        //键盘斜坡,越大减小速度越快(一般要比INC大一点,这样松开键盘能更快为0,太大则会造成底盘停下来的时候跳跃)

#define    REVOLVE_SLOPE_CORGI       50       //底盘扭屁股模式斜坡,越大越快,完成时间越短


//不同模式下的最高速度
#define    REVOLVE_MAX_CORGI         9000//5000     //底盘扭屁股最快速度,太大会让角度过大

void CHAS_Key_Ctrl(void)
{
	if(remot_change == TRUE)//刚从遥控模式切过来,默认为陀螺仪模式
	{
		modeChassis = CHASSIS_GYRO_MODE;
		remot_change = FALSE;
	}
	
	if(GIMBAL_IfBuffHit() == TRUE)//打符模式为机械
	{
		modeChassis = CHASSIS_MECH_MODE;
	}
	
	switch (actChassis)      //SB keil 有警告,键盘模式选择，测试限死是普通模式
	{
		/*------------------扭屁股模式-------------------*/			
		case CHASSIS_CORGI:	
			if(!IF_KEY_PRESSED_F)//F松开
			{
				Chass_Switch_F = 1;
			}
			
			if(IF_KEY_PRESSED_F && Chass_Switch_F == 1)
			{
				Chass_Switch_F = 0;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
			}
			//可以前后移动,按下左右平移、QE、切换机械模式退出
			if(Chass_Key_F_Change)
			{
				modeChassis = CHASSIS_GYRO_MODE;//陀螺仪模式,底盘跟随云台动
				
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);		
			}
			else
			{
				actChassis = CHASSIS_NORMAL;//退出扭屁股模式
			}
			
//			//弹仓开启时要关闭扭腰
//			if((IF_KEY_PRESSED_CTRL || GIMBAL_IfGIMBAL_LEVEL()==TRUE ) && !IF_KEY_PRESSED_F)
//			{
//				Chass_Switch_F = 1;
//				Chass_Key_F_Change ++;
//				Chass_Key_F_Change %= 2;
//				actChassis = CHASSIS_NORMAL;//退出扭屁股模式
//			}
//			else if(IF_KEY_PRESSED_X)//可随时进入45°模式
//			{
//				Chass_Switch_F = 1;
//				Chass_Key_F_Change ++;
//				Chass_Key_F_Change %= 2;
//				actChassis = CHASSIS_PISA;
//			}			
//		break;
//			
		/*-------------打符模式-------------*/
		case CHASSIS_ROSHAN:
						
			if(GIMBAL_IfBuffHit( ) != TRUE)
			{
				actChassis = CHASSIS_NORMAL;//退出打符模式
				modeChassis = CHASSIS_GYRO_MODE;//切回陀螺仪模式
			}	
			else
			{
				modeChassis = CHASSIS_MECH_MODE;//打符底盘进入机械模式
				CHASSIS_REST();//目标速度置0
			}				
		break;
				
//		/*---------------补弹模式,底盘低速----------------*/
//		case CHASSIS_SLOW:
//			if (Magazine_IfOpen() != TRUE)//弹仓关闭
//			{
//				actChassis = CHASSIS_NORMAL;//底盘退出补弹模式
//			}
//			else
//			{
//				modeChassis = CHASSIS_MECH_MODE;//补弹时底盘进入机械模式
//			
//				Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SLOW, TIME_INC_SLOW );
//				Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SLOW );
//			}
//		break;
//				
//		/*-------------手动爬坡模式-------------*/
		case CHASSIS_SZUPUP:
			CHASSIS_SZUPUP_Mode_Ctrl();
		break;
			
//		/*-------------自动闪避模式-------------*/
		case CHASSIS_MISS:
			CHASSIS_MISS_Mode_Ctrl();
		break;
		
//		/*-------------45°对敌模式-------------*/
		case CHASSIS_PISA:
			if(!IF_KEY_PRESSED_X)//F松开
			{
				Chass_Switch_X = 1;
			}
			
			if(IF_KEY_PRESSED_X && Chass_Switch_X == 1)
			{
				Chass_Switch_X = 0;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
			}
			
			if(Chass_Key_X_Change)//前后退出
			{
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_PISA_Mode_Ctrl();
			}
			else
			{
				actChassis = CHASSIS_NORMAL;
			}
			
			//弹仓开启时要关闭
//			if( (IF_KEY_PRESSED_CTRL 
//					|| GIMBAL_IfGIMBAL_LEVEL()==TRUE 
//						|| IF_KEY_PRESSED_W || IF_KEY_PRESSED_S) 
//							&& !IF_KEY_PRESSED_X)
//			{
//				Chass_Switch_X = 1;
//				Chass_Key_X_Change ++;
//				Chass_Key_X_Change %= 2;
//				actChassis = CHASSIS_NORMAL;//退出扭屁股模式
//			}
//			else if(IF_KEY_PRESSED_F)//可随时进入扭腰模式
//			{
//				Chass_Switch_X = 1;
//				Chass_Key_X_Change ++;
//				Chass_Key_X_Change %= 2;
//				actChassis = CHASSIS_CORGI;
//			}
		break;
		case CHASSIS_GYROSCOPE:
			if(!IF_KEY_PRESSED_G)//G松开
			{
				Chass_Switch_G=1;
			}
			else if(IF_KEY_PRESSED_G&&Chass_Switch_G)
			{
				Chass_Switch_G=0;
				Chass_Key_G_Change++;
				Chass_Key_G_Change%=2;
			}
			
			if(Chass_Key_G_Change)
			{
				CHASSIS_GYROSCOPE_Mode_Ctrl(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
			}
			else
			{
				actChassis = CHASSIS_NORMAL;
			}
			
		break;
			
	}
}

/*************************底盘键盘模式各类模式小函数****************************/

/**************斜坡***************/


uint16_t timeInc;//斜坡增加变化时间
uint16_t timeInc_Saltation;//前后方向突变下的斜坡增加量,比正常情况下要小
int16_t   timeXFron,    timeXBack,    timeYLeft,    timeYRigh;//键盘  s  w  d   a

#define    TIME_INC_SLOW             1		  //补弹模式下速度变化快慢
#define    TIME_INC_SZUPUP           3		  //手动爬坡模式下速度变化快慢

#define    TIME_INC_SALTATION        1        //突然变速情况下速度变化快慢

#define    REVOLVE_SLOPE_NORMAL      80       //底盘普通模式斜坡,越大越快,完成时间越短
#define    REVOLVE_SLOPE_CORGI       50       //底盘扭屁股模式斜坡,越大越快,完成时间越短

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//键盘模式下扭头斜坡,主要用在扭屁股模式中
float Slope_Chassis_Revolve_Move;

/**
  * @brief  键盘模式下底盘运动计算
  * @param  速度最大输出量    增加速度(最大293)
  * @retval void
  * @attention  键盘控制前后左右平移,平移无机械和陀螺仪模式之分
  *             需要获取时间来进行斜坡函数计算
  */
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp )
{
	static portTickType  ulCurrentTime = 0;
	static uint32_t  ulDelay = 0;
	float k_rc_z = 1;//根据Z速度调节前后左右平移移速比
	
	Chassis_Standard_Move_Max = sMoveMax;//调整速度限幅,水平移动
	timeInc      = sMoveRamp;
	
	ulCurrentTime = xTaskGetTickCount();//当前系统时间
	
	if(fabs(absolute_chassis_speed.vw) > 800)//扭头速度越快,前后速度越慢,防止转弯半径过大
	{
		k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(absolute_chassis_speed.vw) + 800) * (Chassis_Revolve_Move_Max - fabs(absolute_chassis_speed.vw) + 800) )
					/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
		
		LimtValue_f(&k_rc_z,0,1);
	}
	else
	{
		k_rc_z = 1;
	}
	
	if (ulCurrentTime >= ulDelay)//每10ms变化一次斜坡量
	{
		ulDelay = ulCurrentTime + TIME_STAMP_10MS;


		if(actChassis == CHASSIS_NORMAL && !KEY_PRESSED_OFFSET_SHIFT)//只有一般模式下才判断速度突变情况,防止打滑
		{
			if (IF_KEY_PRESSED_W)//等超级电容出来再测试电容放电时是否要全力加速
			{
				timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
				//前进X是正数,可在此加入超级电容按键判断
				if( absolute_chassis_speed.vx < sMoveMax/2.5 )//转向突变,刚开始的一小段时间斜坡降低,防止轮子打滑浪费功率
				{//利用速度是否到最大速度的1/5来判断不知道是否合理
					timeInc_Saltation = TIME_INC_SALTATION;//以爬坡模式处理速度方向突变
				}
				else			//已经过了打滑时间且轮子有了一定的速度
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//同理
				//后退X是负
				if( absolute_chassis_speed.vx  > (-sMoveMax)/2.5 )//转向突变,刚开始的一小段时间斜坡降低,防止轮子打滑浪费功率
				{
					timeInc_Saltation = TIME_INC_SALTATION;//以爬坡模式处理速度方向突变
				}
				else			//已经过了打滑时间且轮子有了一定的速度
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}
		
			//键盘模式下全向移动,斜坡量计算,注意正负,最大输出量*斜坡比例得到缓慢增加的值,模拟摇杆
			//前后的增加斜坡是变化的
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc_Saltation, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc_Saltation, TIME_DEC_NORMAL ) );

			//左右的增加斜坡跟前后不一样,别搞错
			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc/1.5, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc/1.5, TIME_DEC_NORMAL ) );


			absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//前后计算
			absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//左右计算
		}
		else		//其他模式不需要进行速度方向突变特殊处理
		{
			if (IF_KEY_PRESSED_W)
			{
				timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//同理
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}
			
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc, TIME_DEC_NORMAL ) );

			if(actChassis != CHASSIS_CORGI)
			{
				absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//前后计算
				absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//左右计算
			}
		}
	}
}

/**
  * @brief  鼠标控制底盘旋转,键盘QEC控制快速转圈
  * @param  速度最大输出量 
  * @retval void
  * @attention  鼠标控制左右旋转
  */
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax )
{
//	static int16_t sErrorPrev = 0;//上次偏离误差
	float sErrorReal =IMU_Vale.Yaw-GIMBAL_YAW_ENCODER_MIDDLE1;//yaw偏离中心误差
	
	Chassis_Revolve_Move_Max = sRevolMax;//左右旋转限速
	
	if(modeChassis == CHASSIS_GYRO_MODE)//陀螺仪模式
	{
		AngleLoop(&sErrorReal,8192);//获取实时偏差,用于扭屁股下的底盘位置补偿
		absolute_chassis_speed.vw = PID_Calc(&Chassis_Follow_PID,sErrorReal,kKey_Gyro_Chassis_Revolve);
	}
	else     //机械模式
	{
		absolute_chassis_speed.vw = constrain_float( MOUSE_X_MOVE_SPEED*kKey_Mech_Chassis_Revolve, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);
	}
}



/**************键盘模式辅助函数********************/

/**
  * @brief  底盘键盘斜坡函数
  * @param  判断按键是否被按下, 时间量, 每次增加的量, 一共要减小的量
  * @retval 斜坡比例系数
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
	float  factor = 0;

	
	factor = 0.15 * sqrt( 0.15 * (*time) );  //计算速度斜坡,time累加到296.3斜坡就完成
	
	if (status == 1)//按键被按下
	{
		if (factor < 1)//防止time太大
		{
			*time += inc;
		}
	}
	else		//按键松开
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}

	LimtValue_f( &factor, 0, 1 );//注意一定是float类型限幅

	return factor;  //注意方向
}

/**
  * @brief  底盘键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 底盘键盘控制状态下的所有模式切换都在这
  */


void Chassis_NORMAL_Mode_Ctrl(void)
{
	if(!IF_KEY_PRESSED_F)//F松开
	{
		Chass_Switch_F = 1;
	}
	
	if(!IF_KEY_PRESSED_X)//X松开
	{
		Chass_Switch_X = 1;
	}
	
	if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_F == 1)//F按下,切换成扭屁股(不用一直按F)
	{
		Chass_Switch_F = 0;
		Chass_Key_F_Change ++;
		Chass_Key_F_Change %= 2;
		actChassis = CHASSIS_CORGI;//记得写个能退出扭屁股模式的函数
	}
//	else if(Magazine_IfOpen() == TRUE)//弹仓开启,进入补弹模式
//	{
//		actChassis = CHASSIS_SLOW;
//	}
	else if(IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL)//W Ctrl一起按进入爬坡模式
	{
		actChassis = CHASSIS_SZUPUP;//爬坡模式
	}
	else if(JUDGE_IfArmorHurt() == TRUE && GIMBAL_IfBuffHit() == FALSE)//只要伤害更新且无按键按下,就会进入自动闪避模式
	{
		actChassis = CHASSIS_MISS;//打符时关闭自动闪避
	}
	else if(GIMBAL_IfBuffHit() == TRUE)//打符模式
	{
		actChassis = CHASSIS_ROSHAN;
	}
	else if (IF_KEY_PRESSED_X && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_X == 1)//x按下,切换成45°
	{
		Chass_Switch_X = 0;
		Chass_Key_X_Change ++;
		Chass_Key_X_Change %= 2;
		actChassis = CHASSIS_PISA;
	}
	else 					
	{		
		//打符时强制进入机械模式
		if(IF_KEY_PRESSED_CTRL || GIMBAL_IfBuffHit() == TRUE)      //按住CTRL进入机械模式
		{
			modeChassis = CHASSIS_MECH_MODE;
		}
		else			//松开CTRL进入陀螺仪模式
		{
			modeChassis = CHASSIS_GYRO_MODE;
		}

		//移动速度控制
//		if(Cap_Out_Can_Open() == TRUE)//电容放电
//		{
//			Chassis_Keyboard_Move_Calculate(Omni_SupCap_Max, TIME_INC_NORMAL);//放电时最大速度要变
//			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);
//		}
//		else
//		{
//			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);//设置速度最大值与斜坡时间
//			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);			
//		}
		actChassis=CHASSIS_FOLLOW_GIMBAL;
	}	
}
	


/**************各种模式函数********************/

/**
  * @brief  扭屁股模式(位置不变版)
  * @param  速度最大输出量    增加到最大量所需时间
  * @retval void
  * @attention  不管时间，扭到位了就换向   //*******纽的角度应该是不对的*****************/
  
//扭屁股换向选择
#define    CORGI_BEGIN    0    
#define    CORGI_LEFT     1
#define    CORGI_RIGH     2

uint16_t  stateCorgi = CORGI_BEGIN;//标记往哪扭,默认不扭
bool    IfCorgiChange = FALSE;//是否扭到了另一边
int16_t  corgi_angle_target = 0;//左右目标角度


void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp)
{
		int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;


	Chassis_Revolve_Move_Max = sRevolMax;//最大速度设置
	Slope_Chassis_Revolve_Move = sRevolRamp;//扭头斜坡设置

	sAngleError = FindMinAnglePNY();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//计算角度偏差,机械角度转换成欧拉角,用于前进速度补偿
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//暂存实时X变化
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	absolute_chassis_speed.vx = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	absolute_chassis_speed.vy = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	//秘技:反复横跳......
	switch (stateCorgi)
	{
		case CORGI_BEGIN:	//以后可以试试用个随机(标志位不停取反),来让开始扭头的方向随机	  
			corgi_angle_target = -900;//可改最大移动角度,自动闪避模式下被击打时的扭腰角度
			IfCorgiChange = FALSE;
			stateCorgi    = CORGI_LEFT;
		break;
		
		case CORGI_LEFT:
			corgi_angle_target = -1024;//可改最大移动角度			
			IfCorgiChange = FALSE;

			if (sAngleError < -700)//角度误差大于700
			{
					stateCorgi = CORGI_RIGH;
				  IfCorgiChange = TRUE;//标记可以换向
			}			
		break;
			
		case CORGI_RIGH:		
			corgi_angle_target = 1024;
			IfCorgiChange = FALSE;

			if (sAngleError > 700)//角度误差大于700
			{
				stateCorgi = CORGI_LEFT;
				IfCorgiChange = TRUE;//标记可以换向
			}			
		break;
	}

	absolute_chassis_speed.vw = PID_Calc( &Chassis_Follow_PID, (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}


/**
  * @brief  手动爬坡模式
  * @param  void
  * @retval void
  * @attention  
  */

#define    STANDARD_MAX_SZUPUP       3000//5000//6000//4000//3600	  //手动爬坡模式下水平移动速度
#define    REVOLVE_MAX_SZUPUP        9000     //手动爬坡模式下扭头速度

void CHASSIS_SZUPUP_Mode_Ctrl(void)
{
	if( !IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL)//松开任意一个退出爬坡模式
	{
		actChassis = CHASSIS_NORMAL;//底盘退出爬坡模式
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;//陀螺仪模式
		
		Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SZUPUP, TIME_INC_SZUPUP );
		Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SZUPUP );
	}
}

/**
  * @brief  自动闪避模式
  * @param  void
  * @retval void
  * @attention  
  */

//自动闪避
#define   MISS_MAX_TIME    1000    //自动闪避最大归位时间,单位2*ms
uint32_t  Miss_Mode_Time = 0;//自动闪避已过时长

void CHASSIS_MISS_Mode_Ctrl(void)
{
	int16_t  sAngleError   = 0;
//	sAngleError = GIMBAL_GetOffsetAngle();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//有按键按下或者长时间没再受到攻击则退出自动闪避,保证操作跟手
	if( IF_KEY_PRESSED || Miss_Mode_Time > MISS_MAX_TIME )
	{
		actChassis = CHASSIS_NORMAL;//底盘切换成正常模式
		Miss_Mode_Time = 0;
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;
		
		if(JUDGE_IfArmorHurt() == TRUE 	//装甲板数据更新,即受到新的伤害
			|| IfCorgiChange == FALSE)  //屁股没有扭到旁边
		{
			//开启扭屁股一次
			CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
			Miss_Mode_Time = 0;
		}
		else
		{
			Slope_Chassis_Move_Z = 0;//扭屁股实时输出斜坡
			absolute_chassis_speed.vx = 0;
			absolute_chassis_speed.vy = 0;
			absolute_chassis_speed.vw = PID_Calc( &Chassis_Follow_PID,(sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
			Miss_Mode_Time++;
		}	
	}
}


/**
  * @brief  45°模式
  * @param  void
  * @retval void
  * @attention  //可改最大移动角度    这个值很有可能是错误的导致转不到45度
  */

//陀螺仪模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Gyro_Chassis_Standard, kKey_Gyro_Chassis_Revolve;//平移，旋转

void CHASSIS_PISA_Mode_Ctrl(void)
{
	int16_t  corgi_angle_target = 0;//左右目标角度
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;

	sAngleError = 	FindMinAnglePNY();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//计算角度偏差,机械角度转换成欧拉角,用于前进速度补偿
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//暂存实时X变化
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	absolute_chassis_speed.vx  = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	absolute_chassis_speed.vy  = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	corgi_angle_target = -1024;//可改最大移动角度

	absolute_chassis_speed.vw  = PID_Calc( &Chassis_Follow_PID, (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}

/**
  * @brief  小陀螺模式
  * @param  速度最大输出量    增加速度(最大293)
  * @retval void
  * @attention  
  */
void CHASSIS_GYROSCOPE_Mode_Ctrl(int16_t sMoveMax,int16_t sMoveRamp)
{
//	static portTickType  ulCurrentTime = 0;
//	static uint32_t  ulDelay = 0;
//	float k_rc_z = 1;//根据Z速度调节前后左右平移移速比

//	Chassis_Standard_Move_Max = sMoveMax;//调整速度限幅,水平移动
//	timeInc      = sMoveRamp;
//	
//	ulCurrentTime = xTaskGetTickCount();//当前系统时间
//	
//	
//	if (ulCurrentTime >= ulDelay)//每10ms变化一次斜坡量
//	{
//		ulDelay = ulCurrentTime + TIME_STAMP_10MS;		//其他模式不需要进行速度方向突变特殊处理
//		if (IF_KEY_PRESSED_W)
//		{
//			timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
//		}

//		if (IF_KEY_PRESSED_S)
//		{
//			timeXFron = 0;//同理
//		}

//		if (IF_KEY_PRESSED_D)
//		{
//			timeYRigh = 0;
//		}

//		if (IF_KEY_PRESSED_A)
//		{
//			timeYLeft = 0;
//		}
//		
//		Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
//				Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc, TIME_DEC_NORMAL ) );

//		Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
//				Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc, TIME_DEC_NORMAL ) );

//		Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
//				Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc, TIME_DEC_NORMAL ) );

//		Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
//				Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc, TIME_DEC_NORMAL ) );

//		if(actChassis != CHASSIS_CORGI)
//		{
//			absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//前后计算
//			absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//左右计算
//		}
//	}
//	absolute_chassis_speed.vw=0.006;
}

/**
  * @brief  获取底盘移动模式
  * @param  void
  * @retval TRUE:机械模式    false:陀螺仪模式
  * @attention  
  */
bool CHASSIS_IfActiveMode(void)
{
	if (modeChassis == CHASSIS_MECH_MODE)
	{
		return TRUE;//机械
	}
	else
	{
		return FALSE;//陀螺仪
	}
}

/**
  * @brief  底盘是否处于爬坡模式
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfSZUPUP(void)
{
	if(actChassis == CHASSIS_SZUPUP)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  底盘是否处于扭屁股模式
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfCORGI(void)
{
	if(actChassis == CHASSIS_CORGI)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  底盘是否处于45°模式
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfPISA(void)
{
	if(actChassis == CHASSIS_PISA)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}







