#include "gimbal_task.h"
#include "kalman_filter.h"

uint8_t IMU_Init_Finish=0;
GimbalModeType YawGimbalMode =USEENCODER;
GimbalModeType PitchGimbalMode =USEENCODER;
eGimbalCtrlMode  modeGimbal = CLOUD_GYRO_MODE;    ////云台模式选择//
eGimbalAction  actGimbal = GIMBAL_NORMAL;       ////云台操作模式///

PidTypeDef Gimbal_Yaw_Position_PID;
PidTypeDef Gimbal_Pitch_Position_PID;
const static fp32 Gimbal_Yaw_Speed_pid[3] ={100, 0,30};
const static fp32 Gimbal_Yaw_Position_pid[3]={18,0.002,0};	//IMU位置环pid
const static fp32 Gimbal_Yaw_Encoder_Position_pid[3]={2,0.0001,2};	//encoder位置环pid
const static fp32 Gimbal_Pitch_Speed_pid[3] ={100, 0, 30};
const static fp32 Gimbal_Pitch_Position_pid[3]={2,0,0.1};	//IMU位置环pid
const static fp32 Gimbal_Pitch_Encoder_Position_pid[3]={2,0.0001,2};	//encoder位置环pid
const static fp32 nothing[3]={0,0,0};


extKalman_t GimbalYawSpeedKalman;
extKalman_t GimbalPitchSpeedKalman;
extKalman_t Vision_Distance_Kalman;


float Target_Gimbal_Yaw_Position=0;				//使用IMU时的目标值
float Target_Gimbal_Pitch_Position=0;			//使用IMU时的目标值

/*************卡尔曼滤波**************/
			/*一阶卡尔曼*/    
			//云台角度误差卡尔曼
			extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//定义一个kalman指针    (都是在PID里使用的)
			extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//定义一个kalman指针
			extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//定义一个kalman指针
			extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//定义一个kalman指针

			extKalman_t Vision_Distance_Kalman;

			extKalman_t Gimbal_Buff_Yaw_Error_Kalman;//底盘打符
			extKalman_t Gimbal_Buff_Pitch_Error_Kalman;//

			extKalman_t Gimbal_Buff_Yaw_Error_Gim_Kalman;//云台打符
			extKalman_t Gimbal_Buff_Pitch_Error_Gim_Kalman;//


			/*二阶卡尔曼*/
			#define KF_ANGLE	0
			#define KF_SPEED	1
			#define KF_ACCEL	2

			speed_calc_data_t Vision_Yaw_speed_Struct;
			speed_calc_data_t Vision_Pitch_speed_Struct;

			kalman_filter_init_t yaw_kalman_filter_para = {
				.P_data = {2, 0, 0, 2},
				.A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
				.H_data = {1, 0, 0, 1},
				.Q_data = {1, 0, 0, 1},
				.R_data = {200, 0, 0, 400}//500 1000
			};//初始化yaw的部分kalman参数

			kalman_filter_init_t pitch_kalman_filter_para = {
				.P_data = {2, 0, 0, 2},
				.A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
				.H_data = {1, 0, 0, 1},
				.Q_data = {1, 0, 0, 1},
				.R_data = {200, 0, 0, 400}
			};//初始化pitch的部分kalman参数

			kalman_filter_t yaw_kalman_filter;
			kalman_filter_t pitch_kalman_filter;

			/*自动打弹用的一些标志位*/
			bool Mobility_Prediction_Yaw = FALSE;//预测是否开启标志位
			bool Mobi_Pre_Yaw_Fire = FALSE;//默认预测没到位，禁止开枪

			uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
			uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
			uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖
/********************************************************************************/
			
			
/**************************PID参数***********************************/

				//陀螺仪参数
				float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//陀螺仪角度值
				short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//陀螺仪角速度值

				//机械角度中间变量,从CAN中读取数据
				int16_t  angleMotorPit,  angleMotorYaw; 

				//角度误差
				float Cloud_Angle_Error[2][2];//  pitch/yaw    mech/gyro
				float Cloud_Palstance_Error[2][2];//  pitch/yaw    mech/gyro

				//角速度误差累加和
				float Cloud_Palstance_Error_Sum[2][2];//  pitch/yaw    mech/gyro

				//期望角度
				float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro

				//测量角度
				float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro

				//测量角速度
				float Cloud_Palstance_Measure[2][2];//  pitch/yaw    mech/gyro

				//PID参数
				float Cloud_Angle_kpid[2][2][3];//  pitch/yaw    mech/gyro    kp/ki/kd
				float Cloud_Palstance_kpid[2][2][3];


				//PID
				float  pTermPit[2],       pTermYaw[2][2];//   outer/inner    outer/inner//mech/gyro
				float  iTermPit[2],       iTermYaw[2][2];
				float  pidTermPit[2],     pidTermYaw[2][2];
				
				/**************限幅****************/
				//限制云台电机电流最大输出量
				float PID_Out_Max ;      //限制云台电机电流最大输出量
				float PID_Outter_Max; 
				float PID_Iterm_Max;

/*****************************灵敏度*****************************************/
							//机械模式下比例系数,控制键盘响应速度
							float kKey_Mech_Pitch, kKey_Mech_Yaw;

							//陀螺仪模式下比例系数,控制键盘响应速度
							float kKey_Gyro_Pitch, kKey_Gyro_Yaw;

							float kkey_gyro_yaw = 0.38;



/**********************************自瞄******************************************/
						//自瞄突然开启,卡尔曼滤波开启延时
					uint16_t Auto_KF_Delay = 0;

					float debug_y_sk;// = 38;//35;//30;//移动预测系数,越大预测越多
					float debug_y_sb_sk;//哨兵预测系数
					float debug_y_sb_brig_sk;//桥头哨兵
					float debug_p_sk;//移动预测系数,越大预测越多
					float debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
					float debug_auto_err_p;//pitch角度过大关闭预测
					float debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
					float debug_kf_speed_yl;//yaw速度过低关闭预测
					float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
					float debug_kf_speed_yh;//yaw速度过高关闭预测
					float debug_kf_speed_pl;//pitch速度过低关闭预测
					float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
					float debug_kf_p_angcon;//pitch预测量限幅
					float debug_kf_angle_temp;//预测角度斜坡暂存量

					//误差
					float Auto_Error_Yaw[2];//    now/last
					float Auto_Error_Pitch[2];
					float Auto_Distance;//距离单目

					//基地吊射自瞄
					float Base_Error_Yaw;

					//吊射角度      
						float base_mech_pitch = 3960;
					float Base_Yaw_Comp_Gimbal;//吊射左右校准

					float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度

					/*************卡尔曼滤波**************/
					/*一阶卡尔曼*/
					//云台角度误差卡尔曼

					/*二阶卡尔曼*/
					#define KF_ANGLE	0
					#define KF_SPEED	1
					#define KF_ACCEL	2

					float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值

					speed_calc_data_t Vision_Yaw_speed_Struct;
					speed_calc_data_t Vision_Pitch_speed_Struct;

					kalman_filter_t yaw_kalman_filter;
					kalman_filter_t pitch_kalman_filter;



/**********************手打大符*******************/
					#define CONFIRM_BEGIN		0//刚进入手动打符，随便确认位置
					#define CONFIRM_CENTRE		1//确认圆心
					#define CONFIRM_RADIUS		2//确认半径
					#define CONFIRM_HIGH		3//确认高度
					#define CONFIRM_LOCATION	4//确认位置
					int Manual_Step = CONFIRM_BEGIN;//第一步确定圆心，第二步确定半径，第三步WASD确定位置
					float Manual_Pitch_Comp = 70;//手动抬头补偿
					float Buff_Pitch_Comp;//打符抬头自动补偿
					float Buff_Yaw_Comp;//打符红外左右自动补偿，因为红外可能装歪
					float Buff_Pitch_Comp_Gimbal;//打符抬头补偿,摄像头在云台
					float Buff_Yaw_Comp_Gimbal;

					float Base_Yaw_Comp_Gimbal;//吊射左右校准

					//打符校正镜头畸变
					float Buff_Pitch_Correct_Chassis;
					float Buff_Yaw_Correct_Chassis;
					float Buff_Pitch_Correct_Gimbal;
					float Buff_Yaw_Correct_Gimbal;

					float debug_y_mid;// = 5785;//5797;//视觉调坐标转换
					float debug_p_mid;// = 3450;//3522;//7.15m抬头差72机械角度,飓风电机,600pwm

					float gb_yaw_posit_error = 0;//位置差，用来判断是否移动到位
					float gb_pitch_posit_error = 0;//位置差，用来判断是否移动到位

					//刚进入打符判断
					bool is_firstime_into_buff = TRUE;

/**************斜坡***************/
					float Slope_Mouse_Pitch, Slope_Mouse_Yaw;//摩擦轮开启时抬头快慢

					//键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
					float Mouse_Gyro_Yaw, Mouse_Gyro_Pitch;

					//键盘陀螺仪模式下QEC扭头快慢
					float Slope_Turn_Yaw;
					float Slope_Back_Yaw;

					//开启摩擦轮抬头斜坡
					float Slope_Fric_Pitch;

					//自瞄斜坡
					float Slope_Auto_Yaw;
					float Slope_Auto_Pitch;


/****************云台键盘模式下各小函数辅助变量********************/
					//调头模式角度目标
					float TURNMode_Yaw_Back_Total;//按下C,yaw需要改变的角度值
					float TURNMode_Yaw_Turn_Total;//按下QE,yaw需要改变的角度值,正负代表左右转


void GimbalFun(void const * argument)
{
	portTickType currentTime;
	Gimbal_Yaw_Position_PID.angle_max=360;
	Gimbal_Yaw_Position_PID.angle_min=0;
	Gimbal_Yaw_Position_PID.Dead_Zone=0;
	Gimbal_Yaw_Position_PID.I_Separation=5;
	Gimbal_Yaw_Position_PID.out=0;
	Gimbal_MotorYaw.Motor_PID_Speed.out=0;
	
	Gimbal_Pitch_Position_PID.Dead_Zone=0;
	PID_Init(&Gimbal_Yaw_Position_PID,PID_POSITION,Gimbal_Yaw_Position_pid,400,200);
	PID_Init(&Gimbal_Pitch_Position_PID,PID_POSITION,Gimbal_Pitch_Position_pid,400,200);
	Motor_Init(&Gimbal_MotorYaw,5,Gimbal_Yaw_Encoder_Position_pid,400,200,Gimbal_Yaw_Speed_pid,30000,10000);
	Motor_Init(&Gimbal_MotorPitch,6,Gimbal_Pitch_Encoder_Position_pid,400,200,Gimbal_Pitch_Speed_pid,30000,10000);
	
	KalmanCreate(&GimbalYawSpeedKalman,1,1);
	KalmanCreate(&GimbalPitchSpeedKalman,1,1);
	while(1)
	{
		currentTime = xTaskGetTickCount();//当前系统时间
		if(IMU_Init_Finish)	//IMU初始完毕才能动云台
		{
			if(SystemValue==Starting)
			{
				GimbalOpenInit();				//刚开启，将云台缓慢的移动到正向
				Gimbal_MotorYaw.motor_value->target_angle=GIMBAL_YAW_ENCODER_MIDDLE1;
				Gimbal_MotorPitch.motor_value->target_angle=GIMBAL_PITCH_ENCODER_MIDDLE;//初始化结束之后，应该让云台纯编码器走向正前方中间位置
				SystemValue=Running;			//系统初始化结束
			}
			else
			{
				if(YawGimbalMode==USEENCODER&&(actChassis==CHASSIS_FOLLOW_GIMBAL||actChassis==CHASSIS_CORGI))//YAW轴必须采用陀螺仪模式的情况
				{
					Target_Gimbal_Yaw_Position=IMU_Vale.Yaw;	//避免模式切换时云台乱转
					YawGimbalMode=USEIMU;
				}
				switch(ControlMode)
				{
					case REMOTE:
						RemoteControlGimbal();//遥控模式
						break;
					case KEYBOARD:
						KeyboardControlGimbal();//键盘模式
						break;
				}


			}
		}
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
	}
}



void IMUFun(void const * argument)		//获得板载陀螺仪信息
{
	portTickType currentTime;
	uint32_t count=0;
	while(1)
	{
		currentTime = xTaskGetTickCount();
		if(IMU_Get_Offset())
		{
			count++;
			if(IMU_Init_Finish!=1&&count>500)
				IMU_Init_Finish=1;
			IMU_GetInfo();
		}
		vTaskDelayUntil(&currentTime, 1);//绝对延时
	}
}
/**
  * @brief  求得6020的转速
  * @param  需要求的6020的ID
  * @retval void
  * @attention 
  */

void Cal_6020_speed_dp10ms(int ID)	//用编码器测量6020的速度
{
	uint8_t CAN_ID=ID-1;
	int16_t angle_10ms_diff=(int16_t)moto_CAN[CAN_ID].angle-(int16_t)moto_CAN[CAN_ID].last_angle_pre10ms;
	if(angle_10ms_diff>4096)
		moto_CAN[CAN_ID].speed_dp10ms=(angle_10ms_diff-8192);
	else if(angle_10ms_diff<-4096)
		moto_CAN[CAN_ID].speed_dp10ms=(angle_10ms_diff+8192);
	else
		moto_CAN[CAN_ID].speed_dp10ms=angle_10ms_diff;
	moto_CAN[CAN_ID].last_angle_pre10ms=moto_CAN[CAN_ID].angle;
	moto_CAN[CAN_ID].speed_dp10ms=KalmanFilter(&GimbalYawSpeedKalman,moto_CAN[CAN_ID].speed_dp10ms);
}

/**
  * @brief  刚开机，使云台缓慢的移动到＋y平整位置
  * @param  void
  * @retval void
  * @attention 
  */
void GimbalOpenInit(void)
{
	portTickType  StartTime = xTaskGetTickCount();
	int16_t temp_diff1=0;
	int16_t temp_diff2=0;
	Gimbal_MotorYaw.motor_value->target_angle = Gimbal_MotorYaw.motor_value->angle;
	while(1)	//让电机缓慢的转到初始状态
	{
		temp_diff1=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1;
		temp_diff2=Gimbal_MotorPitch.motor_value->angle-GIMBAL_PITCH_ENCODER_MIDDLE;
		if(temp_diff1>4096)
			temp_diff1-=8192;
		else if(temp_diff1<-4096)
			temp_diff1+=8192;
		if(temp_diff2>4096)
			temp_diff2-=8192;
		else if(temp_diff2<-4096)
			temp_diff2+=8192;
		if(temp_diff1>200&&temp_diff2>200)	
			set_moto5678_current(&hcan1,-2000,-2000,0,0);
		else if(temp_diff1<-200&&temp_diff2>200)
			set_moto5678_current(&hcan1,2000,-2000,0,0);
		else if(temp_diff1>200&&temp_diff2<200)	
			set_moto5678_current(&hcan1,-2000,2000,0,0);
		else if(temp_diff1<200&&temp_diff2<-200)
			set_moto5678_current(&hcan1,2000,2000,0,0);
		else if(temp_diff1>200)
			set_moto5678_current(&hcan1,-2000,0,0,0);
		else if(temp_diff1<-200)
			set_moto5678_current(&hcan1,2000,0,0,0);
		else if(temp_diff2>200)
			set_moto5678_current(&hcan1,0,-2000,0,0);
		else if(temp_diff2<-200)
			set_moto5678_current(&hcan1,0,2000,0,0);
		if((abs(temp_diff2)<200&&abs(temp_diff1)<200)||(xTaskGetTickCount()-StartTime>=5000))	//超过5s没有回到比较正的位置，强制PID调
		{
			set_moto5678_current(&hcan1,0,0,0,0);
			break;
		}
		osDelay(1);
	}
	Gimbal_MotorYaw.motor_value->target_angle=GIMBAL_YAW_ENCODER_MIDDLE1;
	Target_Gimbal_Yaw_Position=IMU_Vale.Yaw;
	Gimbal_MotorPitch.motor_value->target_angle=GIMBAL_PITCH_ENCODER_MIDDLE;
	Target_Gimbal_Pitch_Position=JY901_Vale.Pitch;
}



/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention 
  */

void RemoteControlGimbal(void)
{
	YawGimbalMode=USEENCODER;
	PitchGimbalMode=USEENCODER;
	if(YawGimbalMode==USEENCODER)//纯编码器调YAW轴
	{
		
		Gimbal_MotorYaw.motor_value->target_angle-=(float)rc.ch3/SENSITIVITY_REMOTE_GIMBAL_YAW;//目标编码器角度
		AngleLoop_f(&(Gimbal_MotorYaw.motor_value->target_angle),8192);
		PID_Calc(&Gimbal_MotorYaw.Motor_PID_Position,Gimbal_MotorYaw.motor_value->angle,Gimbal_MotorYaw.motor_value->target_angle);
		Gimbal_MotorYaw.motor_value->target_speed_dp10ms=Gimbal_MotorYaw.Motor_PID_Position.out;//位置环
		PID_Calc(&Gimbal_MotorYaw.Motor_PID_Speed,Gimbal_MotorYaw.motor_value->speed_dp10ms,//速度环
		Gimbal_MotorYaw.motor_value->target_speed_dp10ms);
	}
	else if(YawGimbalMode==USEIMU)//IMU调YAW轴
	{
		Target_Gimbal_Yaw_Position+=(float)rc.ch3/SENSITIVITY_REMOTE_GIMBAL_YAW_IMU;		//目标陀螺仪角度
		//板载陀螺仪YAW轴位置环+电机速度环
		AngleLoop_f(&Target_Gimbal_Yaw_Position,360);
		PID_Calc(&Gimbal_Yaw_Position_PID,IMU_Vale.Yaw,Target_Gimbal_Yaw_Position);
		Gimbal_MotorYaw.motor_value->target_speed_dp10ms=-Gimbal_Yaw_Position_PID.out;
		PID_Calc(&Gimbal_MotorYaw.Motor_PID_Speed,Gimbal_MotorYaw.motor_value->speed_dp10ms,
				Gimbal_MotorYaw.motor_value->target_speed_dp10ms);

	}
	if(PitchGimbalMode==USEENCODER)//纯编码器调PITCH轴
	{
		Gimbal_MotorPitch.motor_value->target_angle-=(float)rc.ch4/SENSITIVITY_REMOTE_GIMBAL_PITCH;//目标编码器角度
		LimtValue_f(&(Gimbal_MotorPitch.motor_value->target_angle),GIMBAL_PITCH_ENCODER_MAX,GIMBAL_PITCH_ENCODER_MIN);
		PID_Calc(&Gimbal_MotorPitch.Motor_PID_Position,Gimbal_MotorPitch.motor_value->angle,Gimbal_MotorPitch.motor_value->target_angle);
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms=Gimbal_MotorPitch.Motor_PID_Position.out;//位置环
		PID_Calc(&Gimbal_MotorPitch.Motor_PID_Speed,Gimbal_MotorPitch.motor_value->speed_dp10ms,//速度环
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms);
	}
	else if(PitchGimbalMode==USEIMU)//纯imu调PITCH轴
	{
		Target_Gimbal_Pitch_Position+=rc.ch4/SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU;//目标编码器角度
		//JY901Pitch轴位置环+电机速度环
		PID_Calc(&Gimbal_Pitch_Position_PID,JY901_Vale.Pitch,Target_Gimbal_Pitch_Position);
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms=-Gimbal_Pitch_Position_PID.out;
		PID_Calc(&Gimbal_MotorPitch.Motor_PID_Speed,Gimbal_MotorPitch.motor_value->speed_dp10ms,
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms);
	}

	set_moto5678_current(&hcan1,Gimbal_MotorYaw.Motor_PID_Speed.out,Gimbal_MotorPitch.Motor_PID_Speed.out,0,0);	//加入CAN发送队列中
}
/**
  * @brief  键盘控制方式
  * @param  void
  * @retval void
  * @attention 
  */
void KeyboardControlGimbal(void)
{
		float fMotorOutput[2] = {0};
//		YawGimbalMode=USEENCODER;   /////用编码器调/
//		PitchGimbalMode=USEENCODER;
		
		Cloud_Angle_Measure[YAW][MECH] = Gimbal_MotorYaw.motor_value->angle;    //得到当前角度//
		Cloud_Angle_Measure[PITCH][MECH]= Gimbal_MotorPitch.motor_value->angle;
	
		GIMBAL_Key_Ctrl();     ///在已选择的模式的控制
	
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			iTermYaw[INNER][GYRO] = 0;
		}
		else
		{
			iTermYaw[INNER][MECH] = 0;
		}
		
		GIMBAL_kPID_Init();//根据操作模式变换kpid,每次都要变
		
		GIMBAL_PositionLoop(); //位置闭环

		//注意一定是发送内环数据,注意方向(正负号)
		fMotorOutput[PITCH] = pidTermPit[INNER];
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = pidTermYaw[INNER][MECH];
		}
		else
		{
			fMotorOutput[YAW]   = pidTermYaw[INNER][GYRO];
		}
		
	
		set_moto5678_current(&hcan1, fMotorOutput[PITCH] , fMotorOutput[YAW] ,0,0);	//加入CAN发送队列中
	
	/////////////////////////不是自瞄模式时需要进行一些改进///////////////////////////////////////
		if(actGimbal != GIMBAL_AUTO)
		{
			//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
			Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
			//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
			pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		}

}

/**
  * @brief  云台参数初始化
  * @param  void
  * @retval void
  * @attention 没有加I会有误差,只在系统启动时调用一次
  */
void GIMBAL_InitArgument(void)
{
	/* 灵敏度,响应快慢 */	
	kKey_Mech_Yaw   = 0;
	kKey_Mech_Pitch = 0.45;
	
	kKey_Gyro_Yaw   = -kkey_gyro_yaw;//-0.38;//注意正负,否则会反向
	kKey_Gyro_Pitch = 0.38;//0.45;
	
	/* 斜坡,变化快慢 */
	Slope_Mouse_Pitch = 15;//20;//鼠标响应,抬头低头速度
	Slope_Mouse_Yaw   = 15;//鼠标响应,扭头速度
	
	Slope_Turn_Yaw = 20;//25;//QE扭头快慢
	Slope_Back_Yaw = 20;//30;//C调头快慢
	
	Slope_Auto_Yaw   = 4;//1;//50;
	Slope_Auto_Pitch = 3;//1;//50;
	
	Slope_Fric_Pitch = 8;
	
	/* 键盘模式选择,默认正常模式 */
	actGimbal = GIMBAL_NORMAL;
	
	/**************限幅****************/
//限制云台电机电流最大输出量
float PID_Out_Max = 4999;     //限制云台电机电流最大输出量
float PID_Outter_Max = 6000;
float PID_Iterm_Max = 3000;
	
	/*--------------自瞄角度补偿初始化----------------*/
		debug_y_sk = 14.8;//80;//72;//80;//移动预测系数,越大预测越多
		debug_y_sb_sk = 10;//105;//120;//140;//
		debug_p_sk = 75;//26;//移动预测系数,越大预测越多
		debug_auto_err_y = 150;//25;//角度过大关闭预测
		debug_auto_err_p = 8;
		debug_kf_delay = 50;//40;//200;//220;//预测延时开启
		debug_kf_speed_yl = 0.35;//0.35;//0.45;//速度过低关闭预测
		debug_kf_speed_yl_sb = 0.3;//
		debug_kf_speed_yh = 25;//4;//6.5;//速度过高关闭预测
		debug_kf_speed_pl = 0.05;//0.25;//0.08;//pitch速度过低关闭预测
		debug_kf_y_angcon = 220;//预测量限幅
		debug_kf_p_angcon = 120;//45;//pitch预测量限幅
		
	//打符
	//云台
	//7.1米
	Buff_Pitch_Comp_Gimbal = -54;//-52;//-25;//家里-52，单项赛-60		
	//8米
	//Buff_Pitch_Comp_Gimbal = -63;
	
	Buff_Yaw_Comp_Gimbal   = -27;//-30;

	Buff_Pitch_Correct_Chassis  = 1;
	Buff_Yaw_Correct_Chassis 	= 1;
	Buff_Pitch_Correct_Gimbal	= 1;
	Buff_Yaw_Correct_Gimbal		= 1;
	
	Base_Yaw_Comp_Gimbal = -27;//-30;
	
	//卡尔曼滤波器初始化
	  /*PID角度误差卡尔曼,一阶*/
	KalmanCreate(&Gimbal_Pitch_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Pitch_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Gimbal_Yaw_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Vision_Distance_Kalman, 1, 2000);
	
	
	KalmanCreate(&Gimbal_Buff_Yaw_Error_Kalman, 1, 0);//底盘打符
	KalmanCreate(&Gimbal_Buff_Pitch_Error_Kalman, 1, 0);//
	
	KalmanCreate(&Gimbal_Buff_Yaw_Error_Gim_Kalman, 1, 0);//云台打符
	KalmanCreate(&Gimbal_Buff_Pitch_Error_Gim_Kalman, 1, 0);//
	
	  /*自瞄卡尔曼滤波,二阶*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	
}
/**
  * @brief  pid计算
  * @param  void
  * @retval void
  * @attention 此处不能改变目标角度,只能用来做限幅和调用PID计算函数
  */
/*- 自瞄 -*/
float  v_y_k = 9/*7.2*//*6*/,  v_y_p = 23/*9*//*8*/,  v_y_i = 800/*25*/;
float gv_p_k = 8/*7.7*//*16*/, gv_p_p = 18/*5.2*/, gv_p_i = 400;
/*- 打符 -*/
float gb_y_k = 3.1/*11*/, gb_y_p = 26/*7.8*/, gb_y_i = 600/*70*/;//外kp 内kp 内ki
float gb_p_k = 3.1/*14*/, gb_p_p = 23/*5*/, gb_p_i = 500/*50*/;
void GIMBAL_kPID_Init(void)
{
	//如果为自瞄模式且找到了目标
	if( actGimbal ==  GIMBAL_AUTO && VisionValue.identify_target == TRUE )
	{
		/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP]=gv_p_k;//7;//7;//7.5;//gv_p_k;//10;
			
			Cloud_Angle_kpid[YAW][MECH][KP]=20;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP]=gv_p_p;//3.7;//4.5;//gv_p_p;//4;
			Cloud_Palstance_kpid[PITCH][MECH][KI]=gv_p_i;//70;//100;//gv_p_i;//80;
			Cloud_Palstance_kpid[PITCH][MECH][KD]=0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]=2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]=0;
			Cloud_Palstance_kpid[YAW][MECH][KD]=0;
			
			/* kPID,陀螺仪模式 */
				//outer
			Cloud_Angle_kpid[YAW][GYRO][KP]=v_y_k;//6;//6.5;//v_y_k;//8;
			
				//inner		
			Cloud_Palstance_kpid[YAW][GYRO][KP]=v_y_p;//8;//v_y_p;//5;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=v_y_i;//100;//v_y_i;//50;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;
	}
	else if( (GIMBAL_IfBuffHit() == TRUE && VisionValue.identify_buff == TRUE) )//打符模式，只有机械
	{
		/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP]=gb_p_k;//14;//gb_p_k;//16;//gb_p_k;//15;
			
			Cloud_Angle_kpid[YAW][MECH][KP]=11;//gb_y_k;//12;//13;//gb_y_k;//15;
		
			Cloud_Angle_kpid[YAW][GYRO][KP]=gb_y_k;//12;//13;//gb_y_k;//15;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP]=gb_p_p;//5;//gb_p_p;//4;//3;//gb_p_p;//3;//3.3;
			Cloud_Palstance_kpid[PITCH][MECH][KI]=gb_p_i;//50;//gb_p_i;//70;//30;//gb_p_i;//20;//0.1;
			Cloud_Palstance_kpid[PITCH][MECH][KD]=0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]=7.8;//gb_y_p;//8;//gb_y_p;//3;
			Cloud_Palstance_kpid[YAW][MECH][KI]=70;//gb_y_i;//125;//50;//gb_y_i;//5;
			Cloud_Palstance_kpid[YAW][MECH][KD]=0;

			Cloud_Palstance_kpid[YAW][GYRO][KP]=gb_y_p;//8;//gb_y_p;//3;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=gb_y_i;//125;//50;//gb_y_i;//5;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;
	}
	else if( GIMBAL_If_Base() == TRUE && VisionValue.identify_target == 8 )
	{
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 4.8;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 200;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;
	}
	//除了自瞄与打符模式的情况
	else
	{
		/* kPID,机械模式 */ 
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP]=10;//12;//15;//16;//16;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]=11;//10;//10;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP]=2.5;//3.8;//4.1;//3;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI]=50;//75;//100;//40;//0.5;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD]=0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]=5.5;//2.5;//2.5;
			Cloud_Palstance_kpid[YAW][MECH][KI]=150;//70;//0.21;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]=0;
			
			/* kPID,陀螺仪模式 */
				//outer		
			Cloud_Angle_kpid[YAW][GYRO][KP]=9.5;
			
				//inner		
			Cloud_Palstance_kpid[YAW][GYRO][KP]=8.5;//9;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=50;//100;//150;//75;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;
	}
}




/***********************************************************************************************
     *  下面的部分是键盘模式
			
				
          

 ****************************************************************************************************/


/**
  * @brief  键盘控制云台模式
  * @param  void
  * @retval void
  * @attention 
  */
	
bool first_time_into_base = TRUE;
	
void GIMBAL_Key_Ctrl(void)
{
	if(modeGimbal == CLOUD_GYRO_MODE)
	{
		//限制云台与底盘分离角度
//		Gimbal_Chass_Separ_Limit();
	}
	
	switch(actGimbal)//SB keil会有警告
	{
		/*--------------云台模式选择----------------*/
		case GIMBAL_NORMAL:
			GIMBAL_NORMAL_Mode_Ctrl();//在此选择控制模式
		break;
		
		/*--------------V  180°调头----------------*/
		case GIMBAL_AROUND:
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式
		
			if (TURNMode_Yaw_Back_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Back_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
		
		/*------------弹仓开启,禁止抬头-----------------*/
		case GIMBAL_LEVEL:
			GIMBAL_LEVEL_Mode_Ctrl();
		break;
		
		/*--------------Q E  90°调头----------------*/
		case GIMBAL_TURN:				
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式

		    if (TURNMode_Yaw_Turn_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Turn_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
			
//		/*--------------右键自瞄----------------*/	
		case GIMBAL_AUTO:
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式
		
			if(!IF_MOUSE_PRESSED_RIGH)//松开右键退出自瞄
			{
				actGimbal = GIMBAL_NORMAL;
				
				//自瞄目标偏差清零,避免切换时云台跳动
				VisionValue.identify_target = FALSE;
				Auto_KF_Delay = 0;//清零给下次延迟预测用
				Mobility_Prediction_Yaw = FALSE;//标记预测没开启
				Mobi_Pre_Yaw_Fire = FALSE;//默认标记预测没到位，禁止开火
				
				mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
				mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
				mobpre_yaw_stop_delay = 0;//停止预测开火延时重置
			}
			else
			{
				GIMBAL_AUTO_Mode_Ctrl();//自瞄控制函数
			}
		break;
		
//		/*--------------Ctrl+V键打小符----------------*/	
		case GIMBAL_SM_BUFF:
			//任意方向移动退出打符
			if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
					|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E)
			{
				actGimbal = GIMBAL_NORMAL;
				modeGimbal = CLOUD_GYRO_MODE;//退出打符切回陀螺仪模式
				
				gb_yaw_posit_error   = 1000;
				gb_pitch_posit_error = 1000;
				
				is_firstime_into_buff = TRUE;
			}
			else
			{
						modeGimbal = CLOUD_GYRO_MODE;//打符时底盘不动
						GIMBAL_BUFF_Mode_Ctrl_Gimbal();//云台打符
				
			}		
		break;	
			
//		/*--------------Ctrl+F键打大符----------------*/	
		case GIMBAL_BUFF:
			//任意方向移动退出打符
			if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
					|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)
			{
				actGimbal = GIMBAL_NORMAL;
				modeGimbal = CLOUD_GYRO_MODE;//退出打符切回陀螺仪模式
				
				gb_yaw_posit_error   = 1000;
				gb_pitch_posit_error = 1000;
				
				is_firstime_into_buff = TRUE;
			}
			else
			{
					modeGimbal = CLOUD_GYRO_MODE;//打符时底盘不动
					GIMBAL_BUFF_Mode_Ctrl_Gimbal();//云台打符
			}		
		break;
			
		/*--------------C键吊射----------------*/
		case GIMBAL_BASE:
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式
		
			if(!IF_KEY_PRESSED_C)//松开右键退出自瞄
			{
				actGimbal = GIMBAL_NORMAL;
				first_time_into_base = TRUE;
			}
			else
			{
				GIMBAL_BASE_Mode_Ctrl();
			}
		break;
		/*--------------Ctrl+G键手打大符----------------*/	
//		case GIMBAL_MANUAL:
//			//QEV退出
//			if(IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)
//			{
//				actGimbal   = GIMBAL_NORMAL;
//				modeGimbal  = CLOUD_GYRO_MODE;//退出打符切回陀螺仪模式
//				Manual_Step = CONFIRM_BEGIN;
//				Manual_Pitch_Comp = 72;//重置补偿
//			}
//			else
//			{
//				modeGimbal = CLOUD_MECH_MODE;//打符时底盘不动
//				GIMBAL_MANUAL_Mode_Ctrl();
//			}		
//		break;
	}
}	

/*******************云台键盘模式各类模式小函数*******************/

/**
  * @brief  云台键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 云台键盘控制状态下的所有模式切换都在这
  * 无模式切换时一直处于此模式
  */
void GIMBAL_NORMAL_Mode_Ctrl(void)
{
		//按键延时响应,防止手贱狂按
	static portTickType  Key_Ctrl_CurrentTime = 0;
	static uint32_t PressV_Time  = 0;//调头,500ms延时响应,1秒最多按2下
	static uint32_t PressQ_Time  = 0;//90°,250ms延时响应,1秒最多按4下
    static uint32_t PressE_Time  = 0;//90°,250ms延时响应,1秒最多按4下
	static uint32_t PressCF_Time  = 0;//打大符,400ms延时响应
//	static uint32_t PressCG_Time  = 0;//手动打符,400ms延时响应
	static uint32_t PressCV_Time  = 0;//打小符,400ms延时响应
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	
	Key_Ctrl_CurrentTime = xTaskGetTickCount( );//获取实时时间,用来做按键延时判断	
	
	
	if ( CHASSIS_IfActiveMode() == TRUE || Magazine_IfOpen() ==	 TRUE)//获取底盘模式,true为机械模式
	{
		modeGimbal = CLOUD_MECH_MODE;
	} 
	else					//注释掉loop中的底盘会令陀螺仪模式失效
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}

	Manual_Step = CONFIRM_BEGIN;//退出手打大符步骤清零
	
	if ( !IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V
					&& Key_Ctrl_CurrentTime > PressV_Time)
	{   //Ctrl不处于按下状态时按V调头
		actGimbal  =  GIMBAL_AROUND;//切换成调头模式

		PressV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms延时防手贱狂按

		if(IF_KEY_PRESSED_A)//AV左调头
		{
			TURNMode_Yaw_Back_Total = 3579;
		}
		else if(IF_KEY_PRESSED_D)//DV右调头
		{
			TURNMode_Yaw_Back_Total = -3579;
		}
		else//默认右调头
		{
				TURNMode_Yaw_Back_Total = -3579;//因为角度放大了20倍,所以是180°*20约等于3579
		}
	}
	/*---------------------------------*/	
	else if ( !IF_KEY_PRESSED_CTRL
				&& ( (IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > PressQ_Time)
					|| (IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > PressE_Time) ) )
	{   //Ctrl不处于按下状态时按Q(左),E(右)90°调头
		actGimbal = GIMBAL_TURN;//切换成快速扭头模式
		
		//注意方向
		if ( IF_KEY_PRESSED_Q)
		{
			PressQ_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = 1789;//Q左转约8192/4度
		}
		else if (IF_KEY_PRESSED_E)
		{
			PressE_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = -1789;//E右转约8192/4度
		}
			
	}	
	/*---------------------------------*/
	else if ( Magazine_IfWait() == TRUE			//弹仓开启或正在开启,云台归中不给动
				|| Magazine_IfOpen() == TRUE )
	{
		actGimbal = GIMBAL_LEVEL;

	}
	/*---------------------------------*/
	else if (IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)//若SW1不在中,则右键自瞄
	{
		actGimbal = GIMBAL_AUTO;

	}
	/*----------------小符-----------------*/
	else if(IF_KEY_PRESSED_V && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCV_Time)//Ctrl+F打符,400ms响应一次
	{
		PressCV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
		actGimbal = GIMBAL_SM_BUFF;
	}
	/*----------------大符-----------------*/
	else if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCF_Time)//Ctrl+F打符,400ms响应一次
	{
		PressCF_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
		actGimbal = GIMBAL_BUFF;
	}
	/*----------------吊射-----------------*/
	else if(IF_KEY_PRESSED_C && !IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)
	{
		actGimbal = GIMBAL_BASE;
	}
	/*---------------------------------*/
//	else if(IF_KEY_PRESSED_G && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCG_Time)//Ctrl+G手动打符,400ms响应一次
//	{
//		PressCG_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_MANUAL;
//	}
	/*---------------------------------*/
	else       //最后做云台角度计算,这是普通模式下的角度计算,优先级最低,所以放最后面
	{
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = GIMBAL_YAW_ENCODER_MIDDLE1;	//yaw保持不动,永远在中间
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
//			Critical_Handle_Init(&Yaw_Gyro_Angle, Cloud_Angle_Measure[YAW][GYRO]);//重置测量角度			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
//			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}
	}
}

/**
  * @brief  补弹模式
  * @param  void
  * @retval void
  * @attention 此模式下禁止控制pitch
  */
void GIMBAL_LEVEL_Mode_Ctrl(void)
{
	modeGimbal = CLOUD_MECH_MODE;//补弹时进入机械模式
	
	//补弹完毕,退出补弹模式
	if( Magazine_IfWait() == FALSE			
			&& Magazine_IfOpen() == FALSE )
	{
		actGimbal = GIMBAL_NORMAL;
	}
	else//补弹未完成,角度固定在中间
	{
		Cloud_Angle_Target[YAW][MECH]   = GIMBAL_YAW_ENCODER_MIDDLE1;
		Cloud_Angle_Target[PITCH][MECH] = GIMBAL_PITCH_ENCODER_MIDDLE;
	}
}





/**********************************************************************************/
/**
  * @brief  自瞄控制函数
  * @param  void
  * @retval void
  * @attention 中间坐标(0,0),左正右负,上负下正
  *            yaw为陀螺仪模式,pitch为机械模式(其实pitch全程都在用机械模式)
  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
  *            新数据一来,目标角度就实时更新
  */

float debug_y_dk = 450;//yaw距离预测比例，越大预测越少
uint32_t Vision_Time[2];// NOW/LAST

int vision_time_js;
float error_yaw_k   = 1;//7.5;//5.6;//2.2;//误差放大
float error_pitch_k = 10;//5;//3;//2.1;//误差放大
float debug_kf_y_angle;//yaw预测暂存
float debug_kf_p_angle;//pitch预测暂存

//根据距离调节预测比例和限幅
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;

float pitch_speed_k = 0;
float kf_pitch_angcon = 0;

float debug_kf_angle_temp;//预测角度斜坡暂存量
float debug_kf_angle_ramp = 20;//预测角度斜坡变化量
float debug_kf_dist;
float debug_dist_bc = 0;
float gim_auto_ramp_y = 5;//10;//刚开启自瞄时缓慢移过去，防止视觉拖影掉帧
float gim_auto_ramp_p = 5;//刚开启自瞄时缓慢移过去，防止视觉拖影掉帧
int js_yaw = 0;
int js_pitch = 0;

float kf_speed_yl = 0;//


void GIMBAL_AUTO_Mode_Ctrl(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	
	static float yaw_angle_raw, pitch_angle_raw;//卡尔曼滤波角度测量值
	static float yaw_angle_ref;//记录目标角度
	static float pitch_angle_ref;//记录目标角度
	
	float kf_delay_open = 0;
	
	Mobility_Prediction_Yaw = FALSE;//默认标记预测没开启
	Mobi_Pre_Yaw_Fire = FALSE;//默认标记预测没到位，禁止开火
	
//	kf_speed_yl = debug_kf_speed_yl;

	//获取角度偏差量,欧拉角类型,过分依赖于视觉的精准度
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	
	Auto_Distance = KalmanFilter(&Vision_Distance_Kalman, Auto_Distance);
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓数据更新↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_If_Update() == TRUE)//视觉数据更新了
	{
		//更新目标角度//记录当前时刻的目标位置,为卡尔曼做准备
		yaw_angle_ref   = Cloud_Angle_Measure[YAW][GYRO]   + Auto_Error_Yaw[NOW]   * error_yaw_k;
		pitch_angle_ref = Cloud_Angle_Measure[PITCH][MECH] + Auto_Error_Pitch[NOW] * error_pitch_k;
		
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
		Vision_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
	}
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑数据更新↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓二阶卡尔曼计算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_Time[NOW] != Vision_Time[LAST])//更新新数据到来的时间
	{
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//计算视觉延迟
		yaw_angle_raw  = yaw_angle_ref;//更新二阶卡尔曼滤波测量值
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	
		//目标速度解算
	if(VisionValue.identify_target == TRUE)//识别到了目标
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);

	}
	else 
	{
		//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		debug_kf_angle_temp = 0;
	}
	if(Chassis_IfCORGI() == TRUE && actChassis == CHASSIS_CORGI )//扭腰且不在打哨兵   //&& GIMBAL_AUTO_PITCH_SB() == FALSE
	{
		kf_delay_open = debug_kf_delay*3.f;
	}
	else 
	{
		kf_delay_open = debug_kf_delay;
	}
	
	////未识别到目标时鼠标可随意控制云台
	if(VisionValue.identify_target == TRUE)//识别到了目标
	{
		Auto_KF_Delay++;//滤波延时开启

		if(VisionValue.auto_too_close == TRUE 
			&& (actChassis != CHASSIS_CORGI) )//目标距离太近，减小预测   // || GIMBAL_AUTO_PITCH_SB() == FALSE
		{
			yaw_speed_k = debug_y_sk;///4.f;//3.f;//预测系数减半
			kf_yaw_angcon = debug_kf_y_angcon;//3.f;//2.f;
			kf_speed_yl = debug_kf_speed_yl;
		}
		else//正常预测量
		{
			if( Chassis_IfCORGI() == TRUE && GIMBAL_AUTO_PITCH_SB() == TRUE )
			{
				yaw_speed_k = debug_y_sb_sk;			
				kf_yaw_angcon = debug_kf_y_angcon;
				kf_speed_yl = debug_kf_speed_yl_sb;
				
				if(IF_KEY_PRESSED_G)
				{
					yaw_speed_k = debug_y_sb_brig_sk;
					kf_yaw_angcon = debug_kf_y_angcon*1.1f;
					kf_speed_yl = debug_kf_speed_yl_sb*0.4f;//0.9f;
				}
			}
			else
			{
				yaw_speed_k = debug_y_sk;
				kf_yaw_angcon = debug_kf_y_angcon;
				kf_speed_yl = debug_kf_speed_yl;
			}
		}
		/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑二阶卡尔曼计算↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
		js_yaw = yaw_kf_result[KF_SPEED]*1000;
		js_pitch = yaw_kf_result[KF_ANGLE]*1000;
		
		/*---------------yaw给个很小的预测,得到yaw的目标角度------------------*/
		/*预测开启条件*/
		if(fabs(Auto_Error_Yaw[NOW]) < debug_auto_err_y//debug看 
				&& Auto_KF_Delay > kf_delay_open 
					&& fabs(yaw_kf_result[KF_SPEED]) >= kf_speed_yl 
						&& fabs(yaw_kf_result[KF_SPEED]) < debug_kf_speed_yh )
		{
			
			if(yaw_kf_result[KF_SPEED]>=0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) * 1;//debug_kf_dist;
			}
			else if(yaw_kf_result[KF_SPEED]<0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) * 1;//debug_kf_dist;			
			}
//			debug_kf_angle_temp = debug_y_sk * yaw_kf_result[KF_SPEED];//此处不需要速度太慢关预测
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -debug_kf_y_angcon, debug_kf_y_angcon);//预测暂存量限幅
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);//预测量缓慢变化
			
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -debug_kf_y_angcon, debug_kf_y_angcon);
			Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle;//debug_y_sk * (yaw_kf_result[KF_SPEED] - debug_kf_speed);//Vision_Gyro_MovProj_Yaw(yaw_kf_result[1]);//yaw_kf_result[0];
			
			/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓预测到位判断↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
			/*yaw_kf_result[1]左移正，右移负，debug看出来的*/
			/*给自动打弹范围做个小标记*/
			if( (yaw_kf_result[KF_SPEED]>0) //目标向左移且误差值显示说目标在右边，则说明预测到位置，可打弹
					&& (Auto_Error_Yaw[NOW] < 0.3f) )
			{
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置

				mobpre_yaw_left_delay++;
				if(mobpre_yaw_left_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
				}
			}
			else if( (yaw_kf_result[KF_SPEED]<0) //目标向右移且误差值显示说目标在左边，则说明预测到位置，可打弹
						&& (Auto_Error_Yaw[NOW] > -0.3f) )
			{
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				
				mobpre_yaw_right_delay++;
				if(mobpre_yaw_right_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//标记预测没到位，禁止开火
				
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
			}
			/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑预测到位判断↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
			Mobility_Prediction_Yaw = TRUE;//标记预测已开启

			mobpre_yaw_stop_delay = 0;//重置静止时的开火延迟
		}
	

		/*---------------pitch给个很小的预测，得到pitch的目标角度------------------*/

		if( Auto_KF_Delay > debug_kf_delay 
				&& fabs(Auto_Error_Pitch[NOW]) < debug_auto_err_p
					&& fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_pl
						&& (GIMBAL_AUTO_PITCH_SB_SK() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE)
							&& VisionValue.distance/100 < 4.4f
		  )
		{	
			if(VisionValue.auto_too_close == TRUE)//目标距离太近，减小预测
			{
				pitch_speed_k = debug_p_sk/2.f;//预测系数减半
				kf_pitch_angcon = debug_kf_p_angcon/1.5f;
			}
			else//正常预测量
			{
				pitch_speed_k = debug_p_sk;
				kf_pitch_angcon = debug_kf_p_angcon;
			}
			
			if(pitch_kf_result[KF_SPEED]>=0)
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - debug_kf_speed_pl);
			}
			else
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + debug_kf_speed_pl);			
			}
			//pitch预测量限幅
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;
		}
		/*预测条件没达到，关闭预测*/
		else
		{
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
		}
	}

	else  
	{
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = GIMBAL_YAW_ENCODER_MIDDLE1;	//yaw保持不动,永远在中间			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
				Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}
		Auto_KF_Delay = 0;	

		mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
		mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
		mobpre_yaw_stop_delay = 0;//停止预测开火延时重置	
	}
}	

float speed_threshold = 5.f;//速度过快
float debug_speed;//左正右负,一般都在1左右,debug看
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度

		if ((S->speed - S->processed_speed) < -speed_threshold)
		{
			S->processed_speed = S->processed_speed - speed_threshold;//速度斜坡变化
		}
		else if ((S->speed - S->processed_speed) > speed_threshold)
		{
			S->processed_speed = S->processed_speed + speed_threshold;//速度斜坡变化
		}

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}


/**
  * @brief  桥头吊射模式
  * @param  void
  * @retval void
  * @attention 像素点，方便调节，只有YAW，PITCH靠操作手，吊射识别到目标发8  
  */            ///只是调节了一下pitch的值///////
void GIMBAL_BASE_Mode_Ctrl(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	static float yaw_base_angle_raw = 0;
	
	float base_yaw_gyro = 0;
	
	Vision_Base_Yaw_Pixel(&Base_Error_Yaw);
	
	if((first_time_into_base == TRUE) && (VisionValue.identify_target == 8) )   //为啥等于8
	{
		Cloud_Angle_Target[PITCH][MECH] = base_mech_pitch;   //吊射角度 
		first_time_into_base = FALSE;
	}
	if(Vision_If_Update() == TRUE)//视觉数据更新了
	{	
		base_yaw_gyro = Base_Error_Yaw + Base_Yaw_Comp_Gimbal;
		
		yaw_base_angle_raw = Cloud_Angle_Measure[YAW][GYRO] + base_yaw_gyro;
		
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
		Vision_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
	}
	
		//未识别到目标时鼠标可随意控制云台
	if(VisionValue.identify_target == 8)//识别到了目标，注意别和自瞄搞混
	{
		//yaw自瞄
		Cloud_Angle_Target[YAW][GYRO] = yaw_base_angle_raw;
		
		//pitch正常控制
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * (kKey_Mech_Pitch/3);
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * (kKey_Gyro_Pitch/3);//pitch仍旧使用机械模式
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}
	}
	else		//未识别到目标,可随意控制云台
	{	
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = GIMBAL_YAW_ENCODER_MIDDLE1;	//yaw保持不动,永远在中间			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}	
	}
}

/**
  * @brief  打符模式，摄像头位于云台
  * @param  void
  * @retval void
  * @attention 红方打红色,蓝方打蓝色,停留3秒激活打符,可能需要关激光
  *  5倍热量冷却,桥面离地1米2,大风车中心离地2米5,内径70cm,外径80cm
  *  pitch仍是机械模式，跟自瞄一样
  */
uint32_t Vision_Buff_Time[2];//测试帧率
int vision_buff_time_js;

float gb_yaw_angle_gim = 0;//获取角度
float gb_pitch_angle_gim = 0;//获取角度
float buff_gimb_ramp_yaw = 72;//120;//国赛72，单项赛120
float buff_gimb_ramp_pitch = 72;//120;
float buff_into_time = 0;
void GIMBAL_BUFF_Mode_Ctrl_Gimbal(void)
{
	float gb_yaw_gyro = 0;
	float gb_pitch_mech = 0;
	static float y_mid = 0;
	static float p_mid = 0;
	
	static float yaw_buff_angle_raw, pitch_buff_angle_raw;//卡尔曼滤波角度测量值
//	static float yaw_buff_angle_ref;//记录目标角度
//	static float pitch_buff_angle_ref;//记录目标角度
	static float shoot_time = 0;
	static float lost_time  = 0;//一段时间没识别到，归中
	
	
	if(is_firstime_into_buff == TRUE)
	{
		is_firstime_into_buff = FALSE;
		buff_into_time = 0;
		
		//记录进入时的角度
		y_mid = Cloud_Angle_Measure[YAW][GYRO];
		p_mid = Cloud_Angle_Target[PITCH][MECH];
	}
	
	if(is_firstime_into_buff == FALSE)//进入一段时间
	{
		buff_into_time++;
	}
	
	/*- 帧率测试 -*/
	if( Vision_If_Update() == TRUE //identify_buff=2也是识别到了
			&& (VisionValue.identify_buff == TRUE || VisionValue.identify_buff == 2) )//视觉数据更新了
	{	
		//像素点
		Vision_Error_Yaw(&gb_yaw_angle_gim);
		Vision_Error_Pitch(&gb_pitch_angle_gim);
		gb_yaw_gyro   = gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal;
		gb_pitch_mech = gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal;//未加抬头补偿
		/******************************************************************/
	
		yaw_buff_angle_raw 	 = Cloud_Angle_Measure[YAW][GYRO]   + gb_yaw_gyro;
		pitch_buff_angle_raw = Cloud_Angle_Measure[PITCH][MECH] + gb_pitch_mech;
		
		Vision_Buff_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
	}
	
		if(Vision_Buff_Time[NOW] != Vision_Buff_Time[LAST])//更新新数据到来的时间
	{
		vision_buff_time_js = Vision_Buff_Time[NOW] - Vision_Buff_Time[LAST];//计算视觉延迟
		Vision_Buff_Time[LAST] = Vision_Buff_Time[NOW];
	}
	/*--------------------------------*/
	//移动,identify_buff为1或2都是识别到
	if(VisionValue.identify_buff == TRUE || VisionValue.identify_buff == 2)
	{
		if(buff_into_time > 100)//第一次进入打符，不给太快响应
		{
			Cloud_Angle_Target[YAW][GYRO]   = RAMP_float(yaw_buff_angle_raw, Cloud_Angle_Measure[YAW][GYRO], buff_gimb_ramp_yaw);
			Cloud_Angle_Target[PITCH][MECH] = RAMP_float(pitch_buff_angle_raw, Cloud_Angle_Measure[PITCH][MECH], buff_gimb_ramp_pitch);
		}
		else
		{
			Cloud_Angle_Target[YAW][GYRO]   = y_mid;
			Cloud_Angle_Target[PITCH][MECH] = p_mid;
		}
		lost_time = 0;
	}
		else
	{
		Cloud_Angle_Target[YAW][GYRO]   += 0;//debug_y_mid;//Mech_Mid_Yaw;
		Cloud_Angle_Target[PITCH][MECH] += 0;//debug_p_mid;//Mech_Mid_Pitch;
		
		lost_time++;
		
		//连续一段时间没识别到
		if(lost_time>300)
		{
			Cloud_Angle_Target[YAW][GYRO]	= y_mid;
			Cloud_Angle_Target[PITCH][MECH] = p_mid;
		}
	}
	

	
	//计算误差，用来判断是否瞄准到位
	if(VisionValue.identify_buff == FALSE)
	{
		shoot_time++;
		if(shoot_time > 100)//连续200MS没识别到,误差加大，防止退出重进后立马打弹
		{
			gb_yaw_posit_error = 1000;
			gb_pitch_posit_error = 1000;
		}
	}
	else
	{
		shoot_time = 0;
		
		if(buff_into_time > 200)
		{
			gb_yaw_posit_error   = Cloud_Angle_Measure[YAW][GYRO]   - yaw_buff_angle_raw;
			gb_pitch_posit_error = Cloud_Angle_Measure[PITCH][MECH] - pitch_buff_angle_raw;
		}
	}

	
}



/**********************************************是否******************************************/

/**
  * @brief  是否开启手动打符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfManulHit(void)
{
	if (actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启自瞄
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfAutoHit(void)
{
   if(actGimbal == GIMBAL_AUTO)//鼠标右键按下
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  yaw轴开启预测的时候云台是否到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 左右各有延迟，换向时记得清零反向和静止时的延迟
  */
bool GIMBAL_MOBPRE_YAW_FIRE(void)
{
	return Mobi_Pre_Yaw_Fire;
}

/**
  * @brief  自瞄yaw轴预测是否已经开启
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfAuto_MobPre_Yaw(void)
{
  if(actGimbal == GIMBAL_AUTO)//鼠标右键按下
	{
		return Mobility_Prediction_Yaw;//TRUE/FALSE
	}
	else//没开自瞄不可能有预测
	{
		return FALSE;
	}
}

/**
  * @brief  打符yaw是否移动到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 
  */
float debug_y_ready = 30;
float debug_pix_y = 0;
bool GIMBAL_BUFF_YAW_READY(void)
{
	debug_pix_y = fabs(gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal);
	if( (fabs(gb_yaw_posit_error) < debug_y_ready) 
			&& (VisionValue.vision_yaw_value.value != 0) 
				&& fabs(gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal) <= 35 )//(VisionRecvData.identify_buff == TRUE) )//识别到了目标
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  打符pitch是否移动到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 
  */
float debug_p_ready = 30;
float debug_pix_p = 0;
bool GIMBAL_BUFF_PITCH_READY(void)
{
	debug_pix_p = fabs(gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal);
	if( (fabs(gb_pitch_posit_error) < debug_p_ready)
			&& (VisionValue.vision_pitch_value.value != 0) 
				&& fabs(gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal) <= 35)//(VisionRecvData.identify_buff == TRUE) )//识别到了目标
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否在自瞄哨兵
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
float down_sb_pitch = 530;//450
float up_sb_pitch   = 100;//

bool GIMBAL_AUTO_PITCH_SB(void)
{
	if( Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN <= down_sb_pitch/*300*/ 
			|| IF_KEY_PRESSED_G)//抬头接近限位
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/**
  * @brief  是否在中等距离自瞄哨兵,加大预测
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
float pitch_sb_error = 0;

bool GIMBAL_AUTO_PITCH_SB_SK(void)
{
	pitch_sb_error = Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN;
	if( (Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN <= down_sb_pitch/*450*//*550*/)
			&& (Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN > up_sb_pitch) )//抬头接近限位
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启打符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfBuffHit(void)
{
	 if (actGimbal == GIMBAL_BUFF || actGimbal == GIMBAL_SM_BUFF|| actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}



/**
  * @brief  是否开启吊射模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_If_Base(void)
{
    if (actGimbal == GIMBAL_BASE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*****************************云台位置PID控制***********************************/

/**
  * @brief  pid计算
  * @param  void
  * @retval void
  * @attention 此处不能改变目标角度,只能用来做限幅和调用PID计算函数
  */
void GIMBAL_PositionLoop(void)
{
	if (modeGimbal == CLOUD_MECH_MODE)//机械模式
	{
		//pitch
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		//pitch角度限制
		Cloud_Angle_Target[PITCH][MECH] = constrain_float( Cloud_Angle_Target[PITCH][MECH], GIMBAL_PITCH_ENCODER_MIN, GIMBAL_PITCH_ENCODER_MAX );
		vPitch_Mech_PositionLoop();
		
		//yaw
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];//机械模式下实时记录陀螺仪角度,防止切换模式时甩头
		//yaw角度限制
		#if	YAW_POSITION == YAW_UP
			Cloud_Angle_Target[YAW][MECH] = constrain_float( Cloud_Angle_Target[YAW][MECH], GIMBAL_YAW_ENCODER_MIN, GIMBAL_YAW_ENCODER_MAX );
		#else
			Cloud_Angle_Target[YAW][MECH] = constrain_float( Cloud_Angle_Target[YAW][MECH], GIMBAL_YAW_ENCODER_MAX,  GIMBAL_YAW_ENCODER_MIN);
		#endif
		vYaw_Mech_PositionLoop();
	}
	
	else if(modeGimbal == CLOUD_GYRO_MODE)//陀螺仪模式
	{
		//pitch
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		//pitch角度限制
		Cloud_Angle_Target[PITCH][MECH] = constrain_float( Cloud_Angle_Target[PITCH][MECH], GIMBAL_PITCH_ENCODER_MIN, GIMBAL_PITCH_ENCODER_MAX );
		vPitch_Mech_PositionLoop();//陀螺仪模式下PITCH用的仍是机械模式
		
		
		//yaw
		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];//陀螺仪模式下实时记录机械角度,防止切换模式时甩头
		
		vYaw_Gyro_PositionLoop();//YAW陀螺仪模式输出
	}
}


/**
  * @brief  pitch机械模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
int js_pid_p = 0;
void vPitch_Mech_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[PITCH][MECH] = Cloud_Angle_Target[PITCH][MECH] - Cloud_Angle_Measure[PITCH][MECH];
	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[PITCH][MECH] = KalmanFilter(&Gimbal_Pitch_Mech_Error_Kalman, Cloud_Angle_Error[PITCH][MECH]);
	//外环输出
	pTermPit[OUTER] = Cloud_Angle_Error[PITCH][MECH] * Cloud_Angle_kpid[PITCH][MECH][KP];
	pidTermPit[OUTER] = pTermPit[OUTER];
	pidTermPit[OUTER] = constrain_float(pidTermPit[OUTER],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[PITCH][MECH] = pidTermPit[OUTER] - Cloud_Palstance_Measure[PITCH][MECH];
	//内环输出
	pTermPit[INNER] = Cloud_Palstance_Error[PITCH][MECH] * Cloud_Palstance_kpid[PITCH][MECH][KP];
	iTermPit[INNER] += Cloud_Palstance_Error[PITCH][MECH] * Cloud_Palstance_kpid[PITCH][MECH][KI] * 0.002f;
	iTermPit[INNER] = constrain_float(iTermPit[INNER], -PID_Iterm_Max, PID_Iterm_Max);

	pidTermPit[INNER] = pTermPit[INNER] + iTermPit[INNER];
	pidTermPit[INNER] = constrain_float(pidTermPit[INNER], -PID_Out_Max, PID_Out_Max);
	js_pid_p = pidTermPit[INNER]*1000;
}

/**
  * @brief  pitch陀螺仪模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
void vPitch_Gyro_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[PITCH][GYRO] = Cloud_Angle_Target[PITCH][GYRO] - Cloud_Angle_Measure[PITCH][GYRO];
	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[PITCH][GYRO] = KalmanFilter(&Gimbal_Pitch_Gyro_Error_Kalman, Cloud_Angle_Error[PITCH][GYRO]);
	//外环输出
	pTermPit[OUTER] = Cloud_Angle_Error[PITCH][GYRO] * Cloud_Angle_kpid[PITCH][GYRO][KP];
	pidTermPit[OUTER] = pTermPit[OUTER];
	pidTermPit[OUTER] = constrain_float(pidTermPit[OUTER],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[PITCH][GYRO] = pidTermPit[OUTER] - Cloud_Palstance_Measure[PITCH][GYRO];
	//内环输出
	pTermPit[INNER] = Cloud_Palstance_Error[PITCH][GYRO] * Cloud_Palstance_kpid[PITCH][GYRO][KP];
	iTermPit[INNER] += Cloud_Palstance_Error[PITCH][GYRO] * Cloud_Palstance_kpid[PITCH][GYRO][KI];
	pidTermPit[INNER] = constrain_float(pidTermPit[INNER], -PID_Out_Max, PID_Out_Max);
}

/**
  * @brief  yaw机械模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
void vYaw_Mech_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[YAW][MECH] = Cloud_Angle_Target[YAW][MECH] - Cloud_Angle_Measure[YAW][MECH];
	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[YAW][MECH] = KalmanFilter(&Gimbal_Yaw_Mech_Error_Kalman, Cloud_Angle_Error[YAW][MECH]);
	//外环输出
	pTermYaw[OUTER][MECH] = Cloud_Angle_Error[YAW][MECH] * Cloud_Angle_kpid[YAW][MECH][KP];
	pidTermYaw[OUTER][MECH] = pTermYaw[OUTER][MECH];
	pidTermYaw[OUTER][MECH] = constrain_float(pidTermYaw[OUTER][MECH],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[YAW][MECH] = pidTermYaw[OUTER][MECH] - Cloud_Palstance_Measure[YAW][MECH];
	//内环输出
	pTermYaw[INNER][MECH]  = Cloud_Palstance_Error[YAW][MECH] * Cloud_Palstance_kpid[YAW][MECH][KP];
	iTermYaw[INNER][MECH] += Cloud_Palstance_Error[YAW][MECH] * Cloud_Palstance_kpid[YAW][MECH][KI] * 0.002f;
	iTermYaw[INNER][MECH]  = constrain_float(iTermYaw[INNER][MECH], -PID_Iterm_Max, PID_Iterm_Max);
	
	pidTermYaw[INNER][MECH] = pTermYaw[INNER][MECH] + iTermYaw[INNER][MECH];
	pidTermYaw[INNER][MECH] = constrain_float(pidTermYaw[INNER][MECH], -PID_Out_Max, PID_Out_Max);
}

/**
  * @brief  yaw陀螺仪模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
void vYaw_Gyro_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[YAW][GYRO] = Cloud_Angle_Target[YAW][GYRO] - Cloud_Angle_Measure[YAW][GYRO];

	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[YAW][GYRO] = KalmanFilter(&Gimbal_Yaw_Gyro_Error_Kalman, Cloud_Angle_Error[YAW][GYRO]);

	//外环输出
	pTermYaw[OUTER][GYRO] = Cloud_Angle_Error[YAW][GYRO] * Cloud_Angle_kpid[YAW][GYRO][KP];
	pidTermYaw[OUTER][GYRO] = pTermYaw[OUTER][GYRO];
	pidTermYaw[OUTER][GYRO] = constrain_float(pidTermYaw[OUTER][GYRO],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[YAW][GYRO] = pidTermYaw[OUTER][GYRO] - Cloud_Palstance_Measure[YAW][GYRO];
	//内环输出
	pTermYaw[INNER][GYRO]  = Cloud_Palstance_Error[YAW][GYRO] * Cloud_Palstance_kpid[YAW][GYRO][KP];
	iTermYaw[INNER][GYRO] += Cloud_Palstance_Error[YAW][GYRO] * Cloud_Palstance_kpid[YAW][GYRO][KI] * 0.002f;
	iTermYaw[INNER][GYRO]  = constrain_float(iTermYaw[INNER][GYRO], -PID_Iterm_Max, PID_Iterm_Max);
	
	pidTermYaw[INNER][GYRO] = pTermYaw[INNER][GYRO] + iTermYaw[INNER][GYRO];
	pidTermYaw[INNER][GYRO] = constrain_float(pidTermYaw[INNER][GYRO], -PID_Out_Max, PID_Out_Max)*(-1);
}


