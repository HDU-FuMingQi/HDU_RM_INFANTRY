#include "gimbal_task.h"
#include "kalman_filter.h"

uint8_t IMU_Init_Finish=0;
GimbalModeType YawGimbalMode =USEENCODER;
GimbalModeType PitchGimbalMode =USEENCODER;
eGimbalCtrlMode  modeGimbal = CLOUD_GYRO_MODE;    ////��̨ģʽѡ��//
eGimbalAction  actGimbal = GIMBAL_NORMAL;       ////��̨����ģʽ///

PidTypeDef Gimbal_Yaw_Position_PID;
PidTypeDef Gimbal_Pitch_Position_PID;
const static fp32 Gimbal_Yaw_Speed_pid[3] ={100, 0,30};
const static fp32 Gimbal_Yaw_Position_pid[3]={18,0.002,0};	//IMUλ�û�pid
const static fp32 Gimbal_Yaw_Encoder_Position_pid[3]={2,0.0001,2};	//encoderλ�û�pid
const static fp32 Gimbal_Pitch_Speed_pid[3] ={100, 0, 30};
const static fp32 Gimbal_Pitch_Position_pid[3]={2,0,0.1};	//IMUλ�û�pid
const static fp32 Gimbal_Pitch_Encoder_Position_pid[3]={2,0.0001,2};	//encoderλ�û�pid
const static fp32 nothing[3]={0,0,0};


extKalman_t GimbalYawSpeedKalman;
extKalman_t GimbalPitchSpeedKalman;
extKalman_t Vision_Distance_Kalman;


float Target_Gimbal_Yaw_Position=0;				//ʹ��IMUʱ��Ŀ��ֵ
float Target_Gimbal_Pitch_Position=0;			//ʹ��IMUʱ��Ŀ��ֵ

/*************�������˲�**************/
			/*һ�׿�����*/    
			//��̨�Ƕ�������
			extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//����һ��kalmanָ��    (������PID��ʹ�õ�)
			extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//����һ��kalmanָ��
			extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//����һ��kalmanָ��
			extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//����һ��kalmanָ��

			extKalman_t Vision_Distance_Kalman;

			extKalman_t Gimbal_Buff_Yaw_Error_Kalman;//���̴��
			extKalman_t Gimbal_Buff_Pitch_Error_Kalman;//

			extKalman_t Gimbal_Buff_Yaw_Error_Gim_Kalman;//��̨���
			extKalman_t Gimbal_Buff_Pitch_Error_Gim_Kalman;//


			/*���׿�����*/
			#define KF_ANGLE	0
			#define KF_SPEED	1
			#define KF_ACCEL	2

			speed_calc_data_t Vision_Yaw_speed_Struct;
			speed_calc_data_t Vision_Pitch_speed_Struct;

			kalman_filter_init_t yaw_kalman_filter_para = {
				.P_data = {2, 0, 0, 2},
				.A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
				.H_data = {1, 0, 0, 1},
				.Q_data = {1, 0, 0, 1},
				.R_data = {200, 0, 0, 400}//500 1000
			};//��ʼ��yaw�Ĳ���kalman����

			kalman_filter_init_t pitch_kalman_filter_para = {
				.P_data = {2, 0, 0, 2},
				.A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
				.H_data = {1, 0, 0, 1},
				.Q_data = {1, 0, 0, 1},
				.R_data = {200, 0, 0, 400}
			};//��ʼ��pitch�Ĳ���kalman����

			kalman_filter_t yaw_kalman_filter;
			kalman_filter_t pitch_kalman_filter;

			/*�Զ����õ�һЩ��־λ*/
			bool Mobility_Prediction_Yaw = FALSE;//Ԥ���Ƿ�����־λ
			bool Mobi_Pre_Yaw_Fire = FALSE;//Ĭ��Ԥ��û��λ����ֹ��ǹ

			uint16_t mobpre_yaw_left_delay = 0;//����Ԥ����ʱ�жϿɿ�������
			uint16_t mobpre_yaw_right_delay = 0;//����Ԥ����ʱ�жϿɿ�������
			uint16_t mobpre_yaw_stop_delay = 0;//Ԥ��ر���ʱ�жϿɿ�������
/********************************************************************************/
			
			
/**************************PID����***********************************/

				//�����ǲ���
				float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//�����ǽǶ�ֵ
				short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//�����ǽ��ٶ�ֵ

				//��е�Ƕ��м����,��CAN�ж�ȡ����
				int16_t  angleMotorPit,  angleMotorYaw; 

				//�Ƕ����
				float Cloud_Angle_Error[2][2];//  pitch/yaw    mech/gyro
				float Cloud_Palstance_Error[2][2];//  pitch/yaw    mech/gyro

				//���ٶ�����ۼӺ�
				float Cloud_Palstance_Error_Sum[2][2];//  pitch/yaw    mech/gyro

				//�����Ƕ�
				float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro

				//�����Ƕ�
				float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro

				//�������ٶ�
				float Cloud_Palstance_Measure[2][2];//  pitch/yaw    mech/gyro

				//PID����
				float Cloud_Angle_kpid[2][2][3];//  pitch/yaw    mech/gyro    kp/ki/kd
				float Cloud_Palstance_kpid[2][2][3];


				//PID
				float  pTermPit[2],       pTermYaw[2][2];//   outer/inner    outer/inner//mech/gyro
				float  iTermPit[2],       iTermYaw[2][2];
				float  pidTermPit[2],     pidTermYaw[2][2];
				
				/**************�޷�****************/
				//������̨���������������
				float PID_Out_Max ;      //������̨���������������
				float PID_Outter_Max; 
				float PID_Iterm_Max;

/*****************************������*****************************************/
							//��еģʽ�±���ϵ��,���Ƽ�����Ӧ�ٶ�
							float kKey_Mech_Pitch, kKey_Mech_Yaw;

							//������ģʽ�±���ϵ��,���Ƽ�����Ӧ�ٶ�
							float kKey_Gyro_Pitch, kKey_Gyro_Yaw;

							float kkey_gyro_yaw = 0.38;



/**********************************����******************************************/
						//����ͻȻ����,�������˲�������ʱ
					uint16_t Auto_KF_Delay = 0;

					float debug_y_sk;// = 38;//35;//30;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
					float debug_y_sb_sk;//�ڱ�Ԥ��ϵ��
					float debug_y_sb_brig_sk;//��ͷ�ڱ�
					float debug_p_sk;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
					float debug_auto_err_y;// = 10;//15;//10;//15;//yaw�Ƕȹ���ر�Ԥ��
					float debug_auto_err_p;//pitch�Ƕȹ���ر�Ԥ��
					float debug_kf_delay;// = 150;//100;//200;//120;//150;//Ԥ����ʱ����
					float debug_kf_speed_yl;//yaw�ٶȹ��͹ر�Ԥ��
					float debug_kf_speed_yl_sb;//̧ͷ���ڱ�ʱ��С��Ϳɿ�Ԥ����
					float debug_kf_speed_yh;//yaw�ٶȹ��߹ر�Ԥ��
					float debug_kf_speed_pl;//pitch�ٶȹ��͹ر�Ԥ��
					float debug_kf_y_angcon;// = 130;//125;//115;//135;//yawԤ�����޷�
					float debug_kf_p_angcon;//pitchԤ�����޷�
					float debug_kf_angle_temp;//Ԥ��Ƕ�б���ݴ���

					//���
					float Auto_Error_Yaw[2];//    now/last
					float Auto_Error_Pitch[2];
					float Auto_Distance;//���뵥Ŀ

					//���ص�������
					float Base_Error_Yaw;

					//����Ƕ�      
						float base_mech_pitch = 3960;
					float Base_Yaw_Comp_Gimbal;//��������У׼

					float *yaw_kf_result, *pitch_kf_result;//���׿������˲����,0�Ƕ� 1�ٶ�

					/*************�������˲�**************/
					/*һ�׿�����*/
					//��̨�Ƕ�������

					/*���׿�����*/
					#define KF_ANGLE	0
					#define KF_SPEED	1
					#define KF_ACCEL	2

					float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//�������˲��ٶȲ���ֵ

					speed_calc_data_t Vision_Yaw_speed_Struct;
					speed_calc_data_t Vision_Pitch_speed_Struct;

					kalman_filter_t yaw_kalman_filter;
					kalman_filter_t pitch_kalman_filter;



/**********************�ִ���*******************/
					#define CONFIRM_BEGIN		0//�ս����ֶ���������ȷ��λ��
					#define CONFIRM_CENTRE		1//ȷ��Բ��
					#define CONFIRM_RADIUS		2//ȷ�ϰ뾶
					#define CONFIRM_HIGH		3//ȷ�ϸ߶�
					#define CONFIRM_LOCATION	4//ȷ��λ��
					int Manual_Step = CONFIRM_BEGIN;//��һ��ȷ��Բ�ģ��ڶ���ȷ���뾶��������WASDȷ��λ��
					float Manual_Pitch_Comp = 70;//�ֶ�̧ͷ����
					float Buff_Pitch_Comp;//���̧ͷ�Զ�����
					float Buff_Yaw_Comp;//������������Զ���������Ϊ�������װ��
					float Buff_Pitch_Comp_Gimbal;//���̧ͷ����,����ͷ����̨
					float Buff_Yaw_Comp_Gimbal;

					float Base_Yaw_Comp_Gimbal;//��������У׼

					//���У����ͷ����
					float Buff_Pitch_Correct_Chassis;
					float Buff_Yaw_Correct_Chassis;
					float Buff_Pitch_Correct_Gimbal;
					float Buff_Yaw_Correct_Gimbal;

					float debug_y_mid;// = 5785;//5797;//�Ӿ�������ת��
					float debug_p_mid;// = 3450;//3522;//7.15m̧ͷ��72��е�Ƕ�,쫷���,600pwm

					float gb_yaw_posit_error = 0;//λ�ò�����ж��Ƿ��ƶ���λ
					float gb_pitch_posit_error = 0;//λ�ò�����ж��Ƿ��ƶ���λ

					//�ս������ж�
					bool is_firstime_into_buff = TRUE;

/**************б��***************/
					float Slope_Mouse_Pitch, Slope_Mouse_Yaw;//Ħ���ֿ���ʱ̧ͷ����

					//����������ģʽ�����ͳ��yawƫ����,��ֵ���Լ�������С,��ֹ˦ͷ����
					float Mouse_Gyro_Yaw, Mouse_Gyro_Pitch;

					//����������ģʽ��QECŤͷ����
					float Slope_Turn_Yaw;
					float Slope_Back_Yaw;

					//����Ħ����̧ͷб��
					float Slope_Fric_Pitch;

					//����б��
					float Slope_Auto_Yaw;
					float Slope_Auto_Pitch;


/****************��̨����ģʽ�¸�С������������********************/
					//��ͷģʽ�Ƕ�Ŀ��
					float TURNMode_Yaw_Back_Total;//����C,yaw��Ҫ�ı�ĽǶ�ֵ
					float TURNMode_Yaw_Turn_Total;//����QE,yaw��Ҫ�ı�ĽǶ�ֵ,������������ת


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
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		if(IMU_Init_Finish)	//IMU��ʼ��ϲ��ܶ���̨
		{
			if(SystemValue==Starting)
			{
				GimbalOpenInit();				//�տ���������̨�������ƶ�������
				Gimbal_MotorYaw.motor_value->target_angle=GIMBAL_YAW_ENCODER_MIDDLE1;
				Gimbal_MotorPitch.motor_value->target_angle=GIMBAL_PITCH_ENCODER_MIDDLE;//��ʼ������֮��Ӧ������̨��������������ǰ���м�λ��
				SystemValue=Running;			//ϵͳ��ʼ������
			}
			else
			{
				if(YawGimbalMode==USEENCODER&&(actChassis==CHASSIS_FOLLOW_GIMBAL||actChassis==CHASSIS_CORGI))//YAW��������������ģʽ�����
				{
					Target_Gimbal_Yaw_Position=IMU_Vale.Yaw;	//����ģʽ�л�ʱ��̨��ת
					YawGimbalMode=USEIMU;
				}
				switch(ControlMode)
				{
					case REMOTE:
						RemoteControlGimbal();//ң��ģʽ
						break;
					case KEYBOARD:
						KeyboardControlGimbal();//����ģʽ
						break;
				}


			}
		}
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}



void IMUFun(void const * argument)		//��ð�����������Ϣ
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
		vTaskDelayUntil(&currentTime, 1);//������ʱ
	}
}
/**
  * @brief  ���6020��ת��
  * @param  ��Ҫ���6020��ID
  * @retval void
  * @attention 
  */

void Cal_6020_speed_dp10ms(int ID)	//�ñ���������6020���ٶ�
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
  * @brief  �տ�����ʹ��̨�������ƶ�����yƽ��λ��
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
	while(1)	//�õ��������ת����ʼ״̬
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
		if((abs(temp_diff2)<200&&abs(temp_diff1)<200)||(xTaskGetTickCount()-StartTime>=5000))	//����5sû�лص��Ƚ�����λ�ã�ǿ��PID��
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
  * @brief  ң�������Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention 
  */

void RemoteControlGimbal(void)
{
	YawGimbalMode=USEENCODER;
	PitchGimbalMode=USEENCODER;
	if(YawGimbalMode==USEENCODER)//����������YAW��
	{
		
		Gimbal_MotorYaw.motor_value->target_angle-=(float)rc.ch3/SENSITIVITY_REMOTE_GIMBAL_YAW;//Ŀ��������Ƕ�
		AngleLoop_f(&(Gimbal_MotorYaw.motor_value->target_angle),8192);
		PID_Calc(&Gimbal_MotorYaw.Motor_PID_Position,Gimbal_MotorYaw.motor_value->angle,Gimbal_MotorYaw.motor_value->target_angle);
		Gimbal_MotorYaw.motor_value->target_speed_dp10ms=Gimbal_MotorYaw.Motor_PID_Position.out;//λ�û�
		PID_Calc(&Gimbal_MotorYaw.Motor_PID_Speed,Gimbal_MotorYaw.motor_value->speed_dp10ms,//�ٶȻ�
		Gimbal_MotorYaw.motor_value->target_speed_dp10ms);
	}
	else if(YawGimbalMode==USEIMU)//IMU��YAW��
	{
		Target_Gimbal_Yaw_Position+=(float)rc.ch3/SENSITIVITY_REMOTE_GIMBAL_YAW_IMU;		//Ŀ�������ǽǶ�
		//����������YAW��λ�û�+����ٶȻ�
		AngleLoop_f(&Target_Gimbal_Yaw_Position,360);
		PID_Calc(&Gimbal_Yaw_Position_PID,IMU_Vale.Yaw,Target_Gimbal_Yaw_Position);
		Gimbal_MotorYaw.motor_value->target_speed_dp10ms=-Gimbal_Yaw_Position_PID.out;
		PID_Calc(&Gimbal_MotorYaw.Motor_PID_Speed,Gimbal_MotorYaw.motor_value->speed_dp10ms,
				Gimbal_MotorYaw.motor_value->target_speed_dp10ms);

	}
	if(PitchGimbalMode==USEENCODER)//����������PITCH��
	{
		Gimbal_MotorPitch.motor_value->target_angle-=(float)rc.ch4/SENSITIVITY_REMOTE_GIMBAL_PITCH;//Ŀ��������Ƕ�
		LimtValue_f(&(Gimbal_MotorPitch.motor_value->target_angle),GIMBAL_PITCH_ENCODER_MAX,GIMBAL_PITCH_ENCODER_MIN);
		PID_Calc(&Gimbal_MotorPitch.Motor_PID_Position,Gimbal_MotorPitch.motor_value->angle,Gimbal_MotorPitch.motor_value->target_angle);
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms=Gimbal_MotorPitch.Motor_PID_Position.out;//λ�û�
		PID_Calc(&Gimbal_MotorPitch.Motor_PID_Speed,Gimbal_MotorPitch.motor_value->speed_dp10ms,//�ٶȻ�
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms);
	}
	else if(PitchGimbalMode==USEIMU)//��imu��PITCH��
	{
		Target_Gimbal_Pitch_Position+=rc.ch4/SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU;//Ŀ��������Ƕ�
		//JY901Pitch��λ�û�+����ٶȻ�
		PID_Calc(&Gimbal_Pitch_Position_PID,JY901_Vale.Pitch,Target_Gimbal_Pitch_Position);
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms=-Gimbal_Pitch_Position_PID.out;
		PID_Calc(&Gimbal_MotorPitch.Motor_PID_Speed,Gimbal_MotorPitch.motor_value->speed_dp10ms,
		Gimbal_MotorPitch.motor_value->target_speed_dp10ms);
	}

	set_moto5678_current(&hcan1,Gimbal_MotorYaw.Motor_PID_Speed.out,Gimbal_MotorPitch.Motor_PID_Speed.out,0,0);	//����CAN���Ͷ�����
}
/**
  * @brief  ���̿��Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention 
  */
void KeyboardControlGimbal(void)
{
		float fMotorOutput[2] = {0};
//		YawGimbalMode=USEENCODER;   /////�ñ�������/
//		PitchGimbalMode=USEENCODER;
		
		Cloud_Angle_Measure[YAW][MECH] = Gimbal_MotorYaw.motor_value->angle;    //�õ���ǰ�Ƕ�//
		Cloud_Angle_Measure[PITCH][MECH]= Gimbal_MotorPitch.motor_value->angle;
	
		GIMBAL_Key_Ctrl();     ///����ѡ���ģʽ�Ŀ���
	
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			iTermYaw[INNER][GYRO] = 0;
		}
		else
		{
			iTermYaw[INNER][MECH] = 0;
		}
		
		GIMBAL_kPID_Init();//���ݲ���ģʽ�任kpid,ÿ�ζ�Ҫ��
		
		GIMBAL_PositionLoop(); //λ�ñջ�

		//ע��һ���Ƿ����ڻ�����,ע�ⷽ��(������)
		fMotorOutput[PITCH] = pidTermPit[INNER];
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = pidTermYaw[INNER][MECH];
		}
		else
		{
			fMotorOutput[YAW]   = pidTermYaw[INNER][GYRO];
		}
		
	
		set_moto5678_current(&hcan1, fMotorOutput[PITCH] , fMotorOutput[YAW] ,0,0);	//����CAN���Ͷ�����
	
	/////////////////////////��������ģʽʱ��Ҫ����һЩ�Ľ�///////////////////////////////////////
		if(actGimbal != GIMBAL_AUTO)
		{
			//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
			Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
			Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
			//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
			yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
			pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		}

}

/**
  * @brief  ��̨������ʼ��
  * @param  void
  * @retval void
  * @attention û�м�I�������,ֻ��ϵͳ����ʱ����һ��
  */
void GIMBAL_InitArgument(void)
{
	/* ������,��Ӧ���� */	
	kKey_Mech_Yaw   = 0;
	kKey_Mech_Pitch = 0.45;
	
	kKey_Gyro_Yaw   = -kkey_gyro_yaw;//-0.38;//ע������,����ᷴ��
	kKey_Gyro_Pitch = 0.38;//0.45;
	
	/* б��,�仯���� */
	Slope_Mouse_Pitch = 15;//20;//�����Ӧ,̧ͷ��ͷ�ٶ�
	Slope_Mouse_Yaw   = 15;//�����Ӧ,Ťͷ�ٶ�
	
	Slope_Turn_Yaw = 20;//25;//QEŤͷ����
	Slope_Back_Yaw = 20;//30;//C��ͷ����
	
	Slope_Auto_Yaw   = 4;//1;//50;
	Slope_Auto_Pitch = 3;//1;//50;
	
	Slope_Fric_Pitch = 8;
	
	/* ����ģʽѡ��,Ĭ������ģʽ */
	actGimbal = GIMBAL_NORMAL;
	
	/**************�޷�****************/
//������̨���������������
float PID_Out_Max = 4999;     //������̨���������������
float PID_Outter_Max = 6000;
float PID_Iterm_Max = 3000;
	
	/*--------------����ǶȲ�����ʼ��----------------*/
		debug_y_sk = 14.8;//80;//72;//80;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
		debug_y_sb_sk = 10;//105;//120;//140;//
		debug_p_sk = 75;//26;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
		debug_auto_err_y = 150;//25;//�Ƕȹ���ر�Ԥ��
		debug_auto_err_p = 8;
		debug_kf_delay = 50;//40;//200;//220;//Ԥ����ʱ����
		debug_kf_speed_yl = 0.35;//0.35;//0.45;//�ٶȹ��͹ر�Ԥ��
		debug_kf_speed_yl_sb = 0.3;//
		debug_kf_speed_yh = 25;//4;//6.5;//�ٶȹ��߹ر�Ԥ��
		debug_kf_speed_pl = 0.05;//0.25;//0.08;//pitch�ٶȹ��͹ر�Ԥ��
		debug_kf_y_angcon = 220;//Ԥ�����޷�
		debug_kf_p_angcon = 120;//45;//pitchԤ�����޷�
		
	//���
	//��̨
	//7.1��
	Buff_Pitch_Comp_Gimbal = -54;//-52;//-25;//����-52��������-60		
	//8��
	//Buff_Pitch_Comp_Gimbal = -63;
	
	Buff_Yaw_Comp_Gimbal   = -27;//-30;

	Buff_Pitch_Correct_Chassis  = 1;
	Buff_Yaw_Correct_Chassis 	= 1;
	Buff_Pitch_Correct_Gimbal	= 1;
	Buff_Yaw_Correct_Gimbal		= 1;
	
	Base_Yaw_Comp_Gimbal = -27;//-30;
	
	//�������˲�����ʼ��
	  /*PID�Ƕ�������,һ��*/
	KalmanCreate(&Gimbal_Pitch_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Pitch_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Gimbal_Yaw_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Vision_Distance_Kalman, 1, 2000);
	
	
	KalmanCreate(&Gimbal_Buff_Yaw_Error_Kalman, 1, 0);//���̴��
	KalmanCreate(&Gimbal_Buff_Pitch_Error_Kalman, 1, 0);//
	
	KalmanCreate(&Gimbal_Buff_Yaw_Error_Gim_Kalman, 1, 0);//��̨���
	KalmanCreate(&Gimbal_Buff_Pitch_Error_Gim_Kalman, 1, 0);//
	
	  /*���鿨�����˲�,����*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	
}
/**
  * @brief  pid����
  * @param  void
  * @retval void
  * @attention �˴����ܸı�Ŀ��Ƕ�,ֻ���������޷��͵���PID���㺯��
  */
/*- ���� -*/
float  v_y_k = 9/*7.2*//*6*/,  v_y_p = 23/*9*//*8*/,  v_y_i = 800/*25*/;
float gv_p_k = 8/*7.7*//*16*/, gv_p_p = 18/*5.2*/, gv_p_i = 400;
/*- ��� -*/
float gb_y_k = 3.1/*11*/, gb_y_p = 26/*7.8*/, gb_y_i = 600/*70*/;//��kp ��kp ��ki
float gb_p_k = 3.1/*14*/, gb_p_p = 23/*5*/, gb_p_i = 500/*50*/;
void GIMBAL_kPID_Init(void)
{
	//���Ϊ����ģʽ���ҵ���Ŀ��
	if( actGimbal ==  GIMBAL_AUTO && VisionValue.identify_target == TRUE )
	{
		/* kPID,��еģʽ */
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
			
			/* kPID,������ģʽ */
				//outer
			Cloud_Angle_kpid[YAW][GYRO][KP]=v_y_k;//6;//6.5;//v_y_k;//8;
			
				//inner		
			Cloud_Palstance_kpid[YAW][GYRO][KP]=v_y_p;//8;//v_y_p;//5;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=v_y_i;//100;//v_y_i;//50;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;
	}
	else if( (GIMBAL_IfBuffHit() == TRUE && VisionValue.identify_buff == TRUE) )//���ģʽ��ֻ�л�е
	{
		/* kPID,��еģʽ */
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
			/* kPID,������ģʽ */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 4.8;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 200;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;
	}
	//������������ģʽ�����
	else
	{
		/* kPID,��еģʽ */ 
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
			
			/* kPID,������ģʽ */
				//outer		
			Cloud_Angle_kpid[YAW][GYRO][KP]=9.5;
			
				//inner		
			Cloud_Palstance_kpid[YAW][GYRO][KP]=8.5;//9;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=50;//100;//150;//75;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;
	}
}




/***********************************************************************************************
     *  ����Ĳ����Ǽ���ģʽ
			
				
          

 ****************************************************************************************************/


/**
  * @brief  ���̿�����̨ģʽ
  * @param  void
  * @retval void
  * @attention 
  */
	
bool first_time_into_base = TRUE;
	
void GIMBAL_Key_Ctrl(void)
{
	if(modeGimbal == CLOUD_GYRO_MODE)
	{
		//������̨����̷���Ƕ�
//		Gimbal_Chass_Separ_Limit();
	}
	
	switch(actGimbal)//SB keil���о���
	{
		/*--------------��̨ģʽѡ��----------------*/
		case GIMBAL_NORMAL:
			GIMBAL_NORMAL_Mode_Ctrl();//�ڴ�ѡ�����ģʽ
		break;
		
		/*--------------V  180���ͷ----------------*/
		case GIMBAL_AROUND:
			modeGimbal = CLOUD_GYRO_MODE;//����������ģʽ
		
			if (TURNMode_Yaw_Back_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Back_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
		
		/*------------���ֿ���,��ֹ̧ͷ-----------------*/
		case GIMBAL_LEVEL:
			GIMBAL_LEVEL_Mode_Ctrl();
		break;
		
		/*--------------Q E  90���ͷ----------------*/
		case GIMBAL_TURN:				
			modeGimbal = CLOUD_GYRO_MODE;//����������ģʽ

		    if (TURNMode_Yaw_Turn_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Turn_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
			
//		/*--------------�Ҽ�����----------------*/	
		case GIMBAL_AUTO:
			modeGimbal = CLOUD_GYRO_MODE;//����������ģʽ
		
			if(!IF_MOUSE_PRESSED_RIGH)//�ɿ��Ҽ��˳�����
			{
				actGimbal = GIMBAL_NORMAL;
				
				//����Ŀ��ƫ������,�����л�ʱ��̨����
				VisionValue.identify_target = FALSE;
				Auto_KF_Delay = 0;//������´��ӳ�Ԥ����
				Mobility_Prediction_Yaw = FALSE;//���Ԥ��û����
				Mobi_Pre_Yaw_Fire = FALSE;//Ĭ�ϱ��Ԥ��û��λ����ֹ����
				
				mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
				mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
				mobpre_yaw_stop_delay = 0;//ֹͣԤ�⿪����ʱ����
			}
			else
			{
				GIMBAL_AUTO_Mode_Ctrl();//������ƺ���
			}
		break;
		
//		/*--------------Ctrl+V����С��----------------*/	
		case GIMBAL_SM_BUFF:
			//���ⷽ���ƶ��˳����
			if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
					|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E)
			{
				actGimbal = GIMBAL_NORMAL;
				modeGimbal = CLOUD_GYRO_MODE;//�˳�����л�������ģʽ
				
				gb_yaw_posit_error   = 1000;
				gb_pitch_posit_error = 1000;
				
				is_firstime_into_buff = TRUE;
			}
			else
			{
						modeGimbal = CLOUD_GYRO_MODE;//���ʱ���̲���
						GIMBAL_BUFF_Mode_Ctrl_Gimbal();//��̨���
				
			}		
		break;	
			
//		/*--------------Ctrl+F������----------------*/	
		case GIMBAL_BUFF:
			//���ⷽ���ƶ��˳����
			if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
					|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)
			{
				actGimbal = GIMBAL_NORMAL;
				modeGimbal = CLOUD_GYRO_MODE;//�˳�����л�������ģʽ
				
				gb_yaw_posit_error   = 1000;
				gb_pitch_posit_error = 1000;
				
				is_firstime_into_buff = TRUE;
			}
			else
			{
					modeGimbal = CLOUD_GYRO_MODE;//���ʱ���̲���
					GIMBAL_BUFF_Mode_Ctrl_Gimbal();//��̨���
			}		
		break;
			
		/*--------------C������----------------*/
		case GIMBAL_BASE:
			modeGimbal = CLOUD_GYRO_MODE;//����������ģʽ
		
			if(!IF_KEY_PRESSED_C)//�ɿ��Ҽ��˳�����
			{
				actGimbal = GIMBAL_NORMAL;
				first_time_into_base = TRUE;
			}
			else
			{
				GIMBAL_BASE_Mode_Ctrl();
			}
		break;
		/*--------------Ctrl+G���ִ���----------------*/	
//		case GIMBAL_MANUAL:
//			//QEV�˳�
//			if(IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)
//			{
//				actGimbal   = GIMBAL_NORMAL;
//				modeGimbal  = CLOUD_GYRO_MODE;//�˳�����л�������ģʽ
//				Manual_Step = CONFIRM_BEGIN;
//				Manual_Pitch_Comp = 72;//���ò���
//			}
//			else
//			{
//				modeGimbal = CLOUD_MECH_MODE;//���ʱ���̲���
//				GIMBAL_MANUAL_Mode_Ctrl();
//			}		
//		break;
	}
}	

/*******************��̨����ģʽ����ģʽС����*******************/

/**
  * @brief  ��̨����ģʽѡ��,������Ӧ
  * @param  void
  * @retval void
  * @attention ��̨���̿���״̬�µ�����ģʽ�л�������
  * ��ģʽ�л�ʱһֱ���ڴ�ģʽ
  */
void GIMBAL_NORMAL_Mode_Ctrl(void)
{
		//������ʱ��Ӧ,��ֹ�ּ���
	static portTickType  Key_Ctrl_CurrentTime = 0;
	static uint32_t PressV_Time  = 0;//��ͷ,500ms��ʱ��Ӧ,1����ఴ2��
	static uint32_t PressQ_Time  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
    static uint32_t PressE_Time  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
	static uint32_t PressCF_Time  = 0;//����,400ms��ʱ��Ӧ
//	static uint32_t PressCG_Time  = 0;//�ֶ����,400ms��ʱ��Ӧ
	static uint32_t PressCV_Time  = 0;//��С��,400ms��ʱ��Ӧ
	static uint32_t Mouse_Yaw_Stop  = 0;//��겻����������Ӧ
	static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������Ӧ
	
	Key_Ctrl_CurrentTime = xTaskGetTickCount( );//��ȡʵʱʱ��,������������ʱ�ж�	
	
	
	if ( CHASSIS_IfActiveMode() == TRUE || Magazine_IfOpen() ==	 TRUE)//��ȡ����ģʽ,trueΪ��еģʽ
	{
		modeGimbal = CLOUD_MECH_MODE;
	} 
	else					//ע�͵�loop�еĵ��̻���������ģʽʧЧ
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}

	Manual_Step = CONFIRM_BEGIN;//�˳��ִ�����������
	
	if ( !IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V
					&& Key_Ctrl_CurrentTime > PressV_Time)
	{   //Ctrl�����ڰ���״̬ʱ��V��ͷ
		actGimbal  =  GIMBAL_AROUND;//�л��ɵ�ͷģʽ

		PressV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms��ʱ���ּ���

		if(IF_KEY_PRESSED_A)//AV���ͷ
		{
			TURNMode_Yaw_Back_Total = 3579;
		}
		else if(IF_KEY_PRESSED_D)//DV�ҵ�ͷ
		{
			TURNMode_Yaw_Back_Total = -3579;
		}
		else//Ĭ���ҵ�ͷ
		{
				TURNMode_Yaw_Back_Total = -3579;//��Ϊ�ǶȷŴ���20��,������180��*20Լ����3579
		}
	}
	/*---------------------------------*/	
	else if ( !IF_KEY_PRESSED_CTRL
				&& ( (IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > PressQ_Time)
					|| (IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > PressE_Time) ) )
	{   //Ctrl�����ڰ���״̬ʱ��Q(��),E(��)90���ͷ
		actGimbal = GIMBAL_TURN;//�л��ɿ���Ťͷģʽ
		
		//ע�ⷽ��
		if ( IF_KEY_PRESSED_Q)
		{
			PressQ_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms��ʱ���ּ���
			
			TURNMode_Yaw_Turn_Total = 1789;//Q��תԼ8192/4��
		}
		else if (IF_KEY_PRESSED_E)
		{
			PressE_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms��ʱ���ּ���
			
			TURNMode_Yaw_Turn_Total = -1789;//E��תԼ8192/4��
		}
			
	}	
	/*---------------------------------*/
	else if ( Magazine_IfWait() == TRUE			//���ֿ��������ڿ���,��̨���в�����
				|| Magazine_IfOpen() == TRUE )
	{
		actGimbal = GIMBAL_LEVEL;

	}
	/*---------------------------------*/
	else if (IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)//��SW1������,���Ҽ�����
	{
		actGimbal = GIMBAL_AUTO;

	}
	/*----------------С��-----------------*/
	else if(IF_KEY_PRESSED_V && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCV_Time)//Ctrl+F���,400ms��Ӧһ��
	{
		PressCV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
		actGimbal = GIMBAL_SM_BUFF;
	}
	/*----------------���-----------------*/
	else if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCF_Time)//Ctrl+F���,400ms��Ӧһ��
	{
		PressCF_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
		actGimbal = GIMBAL_BUFF;
	}
	/*----------------����-----------------*/
	else if(IF_KEY_PRESSED_C && !IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)
	{
		actGimbal = GIMBAL_BASE;
	}
	/*---------------------------------*/
//	else if(IF_KEY_PRESSED_G && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCG_Time)//Ctrl+G�ֶ����,400ms��Ӧһ��
//	{
//		PressCG_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_MANUAL;
//	}
	/*---------------------------------*/
	else       //�������̨�Ƕȼ���,������ͨģʽ�µĽǶȼ���,���ȼ����,���Է������
	{
		if (modeGimbal == CLOUD_MECH_MODE)//��еģʽ
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = GIMBAL_YAW_ENCODER_MIDDLE1;	//yaw���ֲ���,��Զ���м�
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
//			Critical_Handle_Init(&Yaw_Gyro_Angle, Cloud_Angle_Measure[YAW][GYRO]);//���ò����Ƕ�			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//��¼Ŀ��仯�Ƕ�
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch�Ծ�ʹ�û�еģʽ
//			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch�Ծ�ʹ�û�еģʽ
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
  * @brief  ����ģʽ
  * @param  void
  * @retval void
  * @attention ��ģʽ�½�ֹ����pitch
  */
void GIMBAL_LEVEL_Mode_Ctrl(void)
{
	modeGimbal = CLOUD_MECH_MODE;//����ʱ�����еģʽ
	
	//�������,�˳�����ģʽ
	if( Magazine_IfWait() == FALSE			
			&& Magazine_IfOpen() == FALSE )
	{
		actGimbal = GIMBAL_NORMAL;
	}
	else//����δ���,�Ƕȹ̶����м�
	{
		Cloud_Angle_Target[YAW][MECH]   = GIMBAL_YAW_ENCODER_MIDDLE1;
		Cloud_Angle_Target[PITCH][MECH] = GIMBAL_PITCH_ENCODER_MIDDLE;
	}
}





/**********************************************************************************/
/**
  * @brief  ������ƺ���
  * @param  void
  * @retval void
  * @attention �м�����(0,0),�����Ҹ�,�ϸ�����
  *            yawΪ������ģʽ,pitchΪ��еģʽ(��ʵpitchȫ�̶����û�еģʽ)
  *            ֻ�е����������˲��ڵ�ǰʵʱ�Ƕ����ۼ�����Ŀ��Ƕ�
  *            ������һ��,Ŀ��ǶȾ�ʵʱ����
  */

float debug_y_dk = 450;//yaw����Ԥ�������Խ��Ԥ��Խ��
uint32_t Vision_Time[2];// NOW/LAST

int vision_time_js;
float error_yaw_k   = 1;//7.5;//5.6;//2.2;//���Ŵ�
float error_pitch_k = 10;//5;//3;//2.1;//���Ŵ�
float debug_kf_y_angle;//yawԤ���ݴ�
float debug_kf_p_angle;//pitchԤ���ݴ�

//���ݾ������Ԥ��������޷�
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;

float pitch_speed_k = 0;
float kf_pitch_angcon = 0;

float debug_kf_angle_temp;//Ԥ��Ƕ�б���ݴ���
float debug_kf_angle_ramp = 20;//Ԥ��Ƕ�б�±仯��
float debug_kf_dist;
float debug_dist_bc = 0;
float gim_auto_ramp_y = 5;//10;//�տ�������ʱ�����ƹ�ȥ����ֹ�Ӿ���Ӱ��֡
float gim_auto_ramp_p = 5;//�տ�������ʱ�����ƹ�ȥ����ֹ�Ӿ���Ӱ��֡
int js_yaw = 0;
int js_pitch = 0;

float kf_speed_yl = 0;//


void GIMBAL_AUTO_Mode_Ctrl(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//��겻����������Ӧ
	static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������Ӧ
	
	static float yaw_angle_raw, pitch_angle_raw;//�������˲��ǶȲ���ֵ
	static float yaw_angle_ref;//��¼Ŀ��Ƕ�
	static float pitch_angle_ref;//��¼Ŀ��Ƕ�
	
	float kf_delay_open = 0;
	
	Mobility_Prediction_Yaw = FALSE;//Ĭ�ϱ��Ԥ��û����
	Mobi_Pre_Yaw_Fire = FALSE;//Ĭ�ϱ��Ԥ��û��λ����ֹ����
	
//	kf_speed_yl = debug_kf_speed_yl;

	//��ȡ�Ƕ�ƫ����,ŷ��������,�����������Ӿ��ľ�׼��
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	
	Auto_Distance = KalmanFilter(&Vision_Distance_Kalman, Auto_Distance);
	
	/*�����������������������������������������������ݸ��¡�������������������������������������������������������������*/
	if(Vision_If_Update() == TRUE)//�Ӿ����ݸ�����
	{
		//����Ŀ��Ƕ�//��¼��ǰʱ�̵�Ŀ��λ��,Ϊ��������׼��
		yaw_angle_ref   = Cloud_Angle_Measure[YAW][GYRO]   + Auto_Error_Yaw[NOW]   * error_yaw_k;
		pitch_angle_ref = Cloud_Angle_Measure[PITCH][MECH] + Auto_Error_Pitch[NOW] * error_pitch_k;
		
		Vision_Clean_Update_Flag();//һ��Ҫ�ǵ�����,�����һֱִ��
		Vision_Time[NOW] = xTaskGetTickCount();//��ȡ�����ݵ���ʱ��ʱ��
	}
	/*���������������������������������������������������ݸ��¡���������������������������������������������������������*/
	
	/*�����������������������������������������������׿����������������������������������������������������������������������*/
	if(Vision_Time[NOW] != Vision_Time[LAST])//���������ݵ�����ʱ��
	{
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//�����Ӿ��ӳ�
		yaw_angle_raw  = yaw_angle_ref;//���¶��׿������˲�����ֵ
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	
		//Ŀ���ٶȽ���
	if(VisionValue.identify_target == TRUE)//ʶ����Ŀ��
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);

	}
	else 
	{
		//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		debug_kf_angle_temp = 0;
	}
	if(Chassis_IfCORGI() == TRUE && actChassis == CHASSIS_CORGI )//Ť���Ҳ��ڴ��ڱ�   //&& GIMBAL_AUTO_PITCH_SB() == FALSE
	{
		kf_delay_open = debug_kf_delay*3.f;
	}
	else 
	{
		kf_delay_open = debug_kf_delay;
	}
	
	////δʶ��Ŀ��ʱ�������������̨
	if(VisionValue.identify_target == TRUE)//ʶ����Ŀ��
	{
		Auto_KF_Delay++;//�˲���ʱ����

		if(VisionValue.auto_too_close == TRUE 
			&& (actChassis != CHASSIS_CORGI) )//Ŀ�����̫������СԤ��   // || GIMBAL_AUTO_PITCH_SB() == FALSE
		{
			yaw_speed_k = debug_y_sk;///4.f;//3.f;//Ԥ��ϵ������
			kf_yaw_angcon = debug_kf_y_angcon;//3.f;//2.f;
			kf_speed_yl = debug_kf_speed_yl;
		}
		else//����Ԥ����
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
		/*���������������������������������������������������׿������������������������������������������������������������������*/
		js_yaw = yaw_kf_result[KF_SPEED]*1000;
		js_pitch = yaw_kf_result[KF_ANGLE]*1000;
		
		/*---------------yaw������С��Ԥ��,�õ�yaw��Ŀ��Ƕ�------------------*/
		/*Ԥ�⿪������*/
		if(fabs(Auto_Error_Yaw[NOW]) < debug_auto_err_y//debug�� 
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
//			debug_kf_angle_temp = debug_y_sk * yaw_kf_result[KF_SPEED];//�˴�����Ҫ�ٶ�̫����Ԥ��
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -debug_kf_y_angcon, debug_kf_y_angcon);//Ԥ���ݴ����޷�
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);//Ԥ���������仯
			
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -debug_kf_y_angcon, debug_kf_y_angcon);
			Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle;//debug_y_sk * (yaw_kf_result[KF_SPEED] - debug_kf_speed);//Vision_Gyro_MovProj_Yaw(yaw_kf_result[1]);//yaw_kf_result[0];
			
			/*��������������������������������������������Ԥ�⵽λ�жϡ�������������������������������������������������������������*/
			/*yaw_kf_result[1]�����������Ƹ���debug��������*/
			/*���Զ��򵯷�Χ����С���*/
			if( (yaw_kf_result[KF_SPEED]>0) //Ŀ�������������ֵ��ʾ˵Ŀ�����ұߣ���˵��Ԥ�⵽λ�ã��ɴ�
					&& (Auto_Error_Yaw[NOW] < 0.3f) )
			{
				mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����

				mobpre_yaw_left_delay++;
				if(mobpre_yaw_left_delay > 0/*75*/)//�ȶ�150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ���ɿ���
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ��û��λ�����ɿ���
				}
			}
			else if( (yaw_kf_result[KF_SPEED]<0) //Ŀ�������������ֵ��ʾ˵Ŀ������ߣ���˵��Ԥ�⵽λ�ã��ɴ�
						&& (Auto_Error_Yaw[NOW] > -0.3f) )
			{
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				
				mobpre_yaw_right_delay++;
				if(mobpre_yaw_right_delay > 0/*75*/)//�ȶ�150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ���ɿ���
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ��û��λ�����ɿ���
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//���Ԥ��û��λ����ֹ����
				
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����
			}
			/*������������������������������������������������Ԥ�⵽λ�жϡ���������������������������������������������������������*/
			Mobility_Prediction_Yaw = TRUE;//���Ԥ���ѿ���

			mobpre_yaw_stop_delay = 0;//���þ�ֹʱ�Ŀ����ӳ�
		}
	

		/*---------------pitch������С��Ԥ�⣬�õ�pitch��Ŀ��Ƕ�------------------*/

		if( Auto_KF_Delay > debug_kf_delay 
				&& fabs(Auto_Error_Pitch[NOW]) < debug_auto_err_p
					&& fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_pl
						&& (GIMBAL_AUTO_PITCH_SB_SK() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE)
							&& VisionValue.distance/100 < 4.4f
		  )
		{	
			if(VisionValue.auto_too_close == TRUE)//Ŀ�����̫������СԤ��
			{
				pitch_speed_k = debug_p_sk/2.f;//Ԥ��ϵ������
				kf_pitch_angcon = debug_kf_p_angcon/1.5f;
			}
			else//����Ԥ����
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
			//pitchԤ�����޷�
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;
		}
		/*Ԥ������û�ﵽ���ر�Ԥ��*/
		else
		{
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
		}
	}

	else  
	{
		if (modeGimbal == CLOUD_MECH_MODE)//��еģʽ
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = GIMBAL_YAW_ENCODER_MIDDLE1;	//yaw���ֲ���,��Զ���м�			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
				Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//��¼Ŀ��仯�Ƕ�
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch�Ծ�ʹ�û�еģʽ
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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

		mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
		mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
		mobpre_yaw_stop_delay = 0;//ֹͣԤ�⿪����ʱ����	
	}
}	

float speed_threshold = 5.f;//�ٶȹ���
float debug_speed;//�����Ҹ�,һ�㶼��1����,debug��
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//�����ٶ�

		if ((S->speed - S->processed_speed) < -speed_threshold)
		{
			S->processed_speed = S->processed_speed - speed_threshold;//�ٶ�б�±仯
		}
		else if ((S->speed - S->processed_speed) > speed_threshold)
		{
			S->processed_speed = S->processed_speed + speed_threshold;//�ٶ�б�±仯
		}

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//��������ٶ�
}


/**
  * @brief  ��ͷ����ģʽ
  * @param  void
  * @retval void
  * @attention ���ص㣬������ڣ�ֻ��YAW��PITCH�������֣�����ʶ��Ŀ�귢8  
  */            ///ֻ�ǵ�����һ��pitch��ֵ///////
void GIMBAL_BASE_Mode_Ctrl(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//��겻����������Ӧ
	static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������Ӧ
	static float yaw_base_angle_raw = 0;
	
	float base_yaw_gyro = 0;
	
	Vision_Base_Yaw_Pixel(&Base_Error_Yaw);
	
	if((first_time_into_base == TRUE) && (VisionValue.identify_target == 8) )   //Ϊɶ����8
	{
		Cloud_Angle_Target[PITCH][MECH] = base_mech_pitch;   //����Ƕ� 
		first_time_into_base = FALSE;
	}
	if(Vision_If_Update() == TRUE)//�Ӿ����ݸ�����
	{	
		base_yaw_gyro = Base_Error_Yaw + Base_Yaw_Comp_Gimbal;
		
		yaw_base_angle_raw = Cloud_Angle_Measure[YAW][GYRO] + base_yaw_gyro;
		
		Vision_Clean_Update_Flag();//һ��Ҫ�ǵ�����,�����һֱִ��
		Vision_Time[NOW] = xTaskGetTickCount();//��ȡ�����ݵ���ʱ��ʱ��
	}
	
		//δʶ��Ŀ��ʱ�������������̨
	if(VisionValue.identify_target == 8)//ʶ����Ŀ�꣬ע����������
	{
		//yaw����
		Cloud_Angle_Target[YAW][GYRO] = yaw_base_angle_raw;
		
		//pitch��������
		if (modeGimbal == CLOUD_MECH_MODE)//��еģʽ
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * (kKey_Mech_Pitch/3);
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * (kKey_Gyro_Pitch/3);//pitch�Ծ�ʹ�û�еģʽ
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
	else		//δʶ��Ŀ��,�����������̨
	{	
		if (modeGimbal == CLOUD_MECH_MODE)//��еģʽ
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = GIMBAL_YAW_ENCODER_MIDDLE1;	//yaw���ֲ���,��Զ���м�			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//��¼Ŀ��仯�Ƕ�
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch�Ծ�ʹ�û�еģʽ
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
  * @brief  ���ģʽ������ͷλ����̨
  * @param  void
  * @retval void
  * @attention �췽���ɫ,��������ɫ,ͣ��3�뼤����,������Ҫ�ؼ���
  *  5��������ȴ,�������1��2,��糵�������2��5,�ھ�70cm,�⾶80cm
  *  pitch���ǻ�еģʽ��������һ��
  */
uint32_t Vision_Buff_Time[2];//����֡��
int vision_buff_time_js;

float gb_yaw_angle_gim = 0;//��ȡ�Ƕ�
float gb_pitch_angle_gim = 0;//��ȡ�Ƕ�
float buff_gimb_ramp_yaw = 72;//120;//����72��������120
float buff_gimb_ramp_pitch = 72;//120;
float buff_into_time = 0;
void GIMBAL_BUFF_Mode_Ctrl_Gimbal(void)
{
	float gb_yaw_gyro = 0;
	float gb_pitch_mech = 0;
	static float y_mid = 0;
	static float p_mid = 0;
	
	static float yaw_buff_angle_raw, pitch_buff_angle_raw;//�������˲��ǶȲ���ֵ
//	static float yaw_buff_angle_ref;//��¼Ŀ��Ƕ�
//	static float pitch_buff_angle_ref;//��¼Ŀ��Ƕ�
	static float shoot_time = 0;
	static float lost_time  = 0;//һ��ʱ��ûʶ�𵽣�����
	
	
	if(is_firstime_into_buff == TRUE)
	{
		is_firstime_into_buff = FALSE;
		buff_into_time = 0;
		
		//��¼����ʱ�ĽǶ�
		y_mid = Cloud_Angle_Measure[YAW][GYRO];
		p_mid = Cloud_Angle_Target[PITCH][MECH];
	}
	
	if(is_firstime_into_buff == FALSE)//����һ��ʱ��
	{
		buff_into_time++;
	}
	
	/*- ֡�ʲ��� -*/
	if( Vision_If_Update() == TRUE //identify_buff=2Ҳ��ʶ����
			&& (VisionValue.identify_buff == TRUE || VisionValue.identify_buff == 2) )//�Ӿ����ݸ�����
	{	
		//���ص�
		Vision_Error_Yaw(&gb_yaw_angle_gim);
		Vision_Error_Pitch(&gb_pitch_angle_gim);
		gb_yaw_gyro   = gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal;
		gb_pitch_mech = gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal;//δ��̧ͷ����
		/******************************************************************/
	
		yaw_buff_angle_raw 	 = Cloud_Angle_Measure[YAW][GYRO]   + gb_yaw_gyro;
		pitch_buff_angle_raw = Cloud_Angle_Measure[PITCH][MECH] + gb_pitch_mech;
		
		Vision_Buff_Time[NOW] = xTaskGetTickCount();//��ȡ�����ݵ���ʱ��ʱ��
		Vision_Clean_Update_Flag();//һ��Ҫ�ǵ�����,�����һֱִ��
	}
	
		if(Vision_Buff_Time[NOW] != Vision_Buff_Time[LAST])//���������ݵ�����ʱ��
	{
		vision_buff_time_js = Vision_Buff_Time[NOW] - Vision_Buff_Time[LAST];//�����Ӿ��ӳ�
		Vision_Buff_Time[LAST] = Vision_Buff_Time[NOW];
	}
	/*--------------------------------*/
	//�ƶ�,identify_buffΪ1��2����ʶ��
	if(VisionValue.identify_buff == TRUE || VisionValue.identify_buff == 2)
	{
		if(buff_into_time > 100)//��һ�ν�����������̫����Ӧ
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
		
		//����һ��ʱ��ûʶ��
		if(lost_time>300)
		{
			Cloud_Angle_Target[YAW][GYRO]	= y_mid;
			Cloud_Angle_Target[PITCH][MECH] = p_mid;
		}
	}
	

	
	//�����������ж��Ƿ���׼��λ
	if(VisionValue.identify_buff == FALSE)
	{
		shoot_time++;
		if(shoot_time > 100)//����200MSûʶ��,���Ӵ󣬷�ֹ�˳��ؽ��������
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



/**********************************************�Ƿ�******************************************/

/**
  * @brief  �Ƿ����ֶ����ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
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
  * @brief  �Ƿ�������
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfAutoHit(void)
{
   if(actGimbal == GIMBAL_AUTO)//����Ҽ�����
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  yaw�Ὺ��Ԥ���ʱ����̨�Ƿ�λ
  * @param  void
  * @retval TRUE��λ�ɴ�   FALSEû��λ��ֹ��
  * @attention ���Ҹ����ӳ٣�����ʱ�ǵ����㷴��;�ֹʱ���ӳ�
  */
bool GIMBAL_MOBPRE_YAW_FIRE(void)
{
	return Mobi_Pre_Yaw_Fire;
}

/**
  * @brief  ����yaw��Ԥ���Ƿ��Ѿ�����
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfAuto_MobPre_Yaw(void)
{
  if(actGimbal == GIMBAL_AUTO)//����Ҽ�����
	{
		return Mobility_Prediction_Yaw;//TRUE/FALSE
	}
	else//û�����鲻������Ԥ��
	{
		return FALSE;
	}
}

/**
  * @brief  ���yaw�Ƿ��ƶ���λ
  * @param  void
  * @retval TRUE��λ�ɴ�   FALSEû��λ��ֹ��
  * @attention 
  */
float debug_y_ready = 30;
float debug_pix_y = 0;
bool GIMBAL_BUFF_YAW_READY(void)
{
	debug_pix_y = fabs(gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal);
	if( (fabs(gb_yaw_posit_error) < debug_y_ready) 
			&& (VisionValue.vision_yaw_value.value != 0) 
				&& fabs(gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal) <= 35 )//(VisionRecvData.identify_buff == TRUE) )//ʶ����Ŀ��
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  ���pitch�Ƿ��ƶ���λ
  * @param  void
  * @retval TRUE��λ�ɴ�   FALSEû��λ��ֹ��
  * @attention 
  */
float debug_p_ready = 30;
float debug_pix_p = 0;
bool GIMBAL_BUFF_PITCH_READY(void)
{
	debug_pix_p = fabs(gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal);
	if( (fabs(gb_pitch_posit_error) < debug_p_ready)
			&& (VisionValue.vision_pitch_value.value != 0) 
				&& fabs(gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal) <= 35)//(VisionRecvData.identify_buff == TRUE) )//ʶ����Ŀ��
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �Ƿ��������ڱ�
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
float down_sb_pitch = 530;//450
float up_sb_pitch   = 100;//

bool GIMBAL_AUTO_PITCH_SB(void)
{
	if( Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN <= down_sb_pitch/*300*/ 
			|| IF_KEY_PRESSED_G)//̧ͷ�ӽ���λ
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/**
  * @brief  �Ƿ����еȾ��������ڱ�,�Ӵ�Ԥ��
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
float pitch_sb_error = 0;

bool GIMBAL_AUTO_PITCH_SB_SK(void)
{
	pitch_sb_error = Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN;
	if( (Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN <= down_sb_pitch/*450*//*550*/)
			&& (Cloud_Angle_Measure[PITCH][MECH] - GIMBAL_PITCH_ENCODER_MIN > up_sb_pitch) )//̧ͷ�ӽ���λ
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �Ƿ������ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
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
  * @brief  �Ƿ�������ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
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


/*****************************��̨λ��PID����***********************************/

/**
  * @brief  pid����
  * @param  void
  * @retval void
  * @attention �˴����ܸı�Ŀ��Ƕ�,ֻ���������޷��͵���PID���㺯��
  */
void GIMBAL_PositionLoop(void)
{
	if (modeGimbal == CLOUD_MECH_MODE)//��еģʽ
	{
		//pitch
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		//pitch�Ƕ�����
		Cloud_Angle_Target[PITCH][MECH] = constrain_float( Cloud_Angle_Target[PITCH][MECH], GIMBAL_PITCH_ENCODER_MIN, GIMBAL_PITCH_ENCODER_MAX );
		vPitch_Mech_PositionLoop();
		
		//yaw
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];//��еģʽ��ʵʱ��¼�����ǽǶ�,��ֹ�л�ģʽʱ˦ͷ
		//yaw�Ƕ�����
		#if	YAW_POSITION == YAW_UP
			Cloud_Angle_Target[YAW][MECH] = constrain_float( Cloud_Angle_Target[YAW][MECH], GIMBAL_YAW_ENCODER_MIN, GIMBAL_YAW_ENCODER_MAX );
		#else
			Cloud_Angle_Target[YAW][MECH] = constrain_float( Cloud_Angle_Target[YAW][MECH], GIMBAL_YAW_ENCODER_MAX,  GIMBAL_YAW_ENCODER_MIN);
		#endif
		vYaw_Mech_PositionLoop();
	}
	
	else if(modeGimbal == CLOUD_GYRO_MODE)//������ģʽ
	{
		//pitch
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		//pitch�Ƕ�����
		Cloud_Angle_Target[PITCH][MECH] = constrain_float( Cloud_Angle_Target[PITCH][MECH], GIMBAL_PITCH_ENCODER_MIN, GIMBAL_PITCH_ENCODER_MAX );
		vPitch_Mech_PositionLoop();//������ģʽ��PITCH�õ����ǻ�еģʽ
		
		
		//yaw
		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];//������ģʽ��ʵʱ��¼��е�Ƕ�,��ֹ�л�ģʽʱ˦ͷ
		
		vYaw_Gyro_PositionLoop();//YAW������ģʽ���
	}
}


/**
  * @brief  pitch��еģʽ
  * @param  void
  * @retval void
  * @attention ����PID�����
  */
int js_pid_p = 0;
void vPitch_Mech_PositionLoop(void)
{
	//�Ƕ����
	Cloud_Angle_Error[PITCH][MECH] = Cloud_Angle_Target[PITCH][MECH] - Cloud_Angle_Measure[PITCH][MECH];
	//�����п������˲�,������Ƶ�ͷ��ȶ���
	Cloud_Angle_Error[PITCH][MECH] = KalmanFilter(&Gimbal_Pitch_Mech_Error_Kalman, Cloud_Angle_Error[PITCH][MECH]);
	//�⻷���
	pTermPit[OUTER] = Cloud_Angle_Error[PITCH][MECH] * Cloud_Angle_kpid[PITCH][MECH][KP];
	pidTermPit[OUTER] = pTermPit[OUTER];
	pidTermPit[OUTER] = constrain_float(pidTermPit[OUTER],-PID_Outter_Max,PID_Outter_Max);
	
	//���ٶ����
	Cloud_Palstance_Error[PITCH][MECH] = pidTermPit[OUTER] - Cloud_Palstance_Measure[PITCH][MECH];
	//�ڻ����
	pTermPit[INNER] = Cloud_Palstance_Error[PITCH][MECH] * Cloud_Palstance_kpid[PITCH][MECH][KP];
	iTermPit[INNER] += Cloud_Palstance_Error[PITCH][MECH] * Cloud_Palstance_kpid[PITCH][MECH][KI] * 0.002f;
	iTermPit[INNER] = constrain_float(iTermPit[INNER], -PID_Iterm_Max, PID_Iterm_Max);

	pidTermPit[INNER] = pTermPit[INNER] + iTermPit[INNER];
	pidTermPit[INNER] = constrain_float(pidTermPit[INNER], -PID_Out_Max, PID_Out_Max);
	js_pid_p = pidTermPit[INNER]*1000;
}

/**
  * @brief  pitch������ģʽ
  * @param  void
  * @retval void
  * @attention ����PID�����
  */
void vPitch_Gyro_PositionLoop(void)
{
	//�Ƕ����
	Cloud_Angle_Error[PITCH][GYRO] = Cloud_Angle_Target[PITCH][GYRO] - Cloud_Angle_Measure[PITCH][GYRO];
	//�����п������˲�,������Ƶ�ͷ��ȶ���
	Cloud_Angle_Error[PITCH][GYRO] = KalmanFilter(&Gimbal_Pitch_Gyro_Error_Kalman, Cloud_Angle_Error[PITCH][GYRO]);
	//�⻷���
	pTermPit[OUTER] = Cloud_Angle_Error[PITCH][GYRO] * Cloud_Angle_kpid[PITCH][GYRO][KP];
	pidTermPit[OUTER] = pTermPit[OUTER];
	pidTermPit[OUTER] = constrain_float(pidTermPit[OUTER],-PID_Outter_Max,PID_Outter_Max);
	
	//���ٶ����
	Cloud_Palstance_Error[PITCH][GYRO] = pidTermPit[OUTER] - Cloud_Palstance_Measure[PITCH][GYRO];
	//�ڻ����
	pTermPit[INNER] = Cloud_Palstance_Error[PITCH][GYRO] * Cloud_Palstance_kpid[PITCH][GYRO][KP];
	iTermPit[INNER] += Cloud_Palstance_Error[PITCH][GYRO] * Cloud_Palstance_kpid[PITCH][GYRO][KI];
	pidTermPit[INNER] = constrain_float(pidTermPit[INNER], -PID_Out_Max, PID_Out_Max);
}

/**
  * @brief  yaw��еģʽ
  * @param  void
  * @retval void
  * @attention ����PID�����
  */
void vYaw_Mech_PositionLoop(void)
{
	//�Ƕ����
	Cloud_Angle_Error[YAW][MECH] = Cloud_Angle_Target[YAW][MECH] - Cloud_Angle_Measure[YAW][MECH];
	//�����п������˲�,������Ƶ�ͷ��ȶ���
	Cloud_Angle_Error[YAW][MECH] = KalmanFilter(&Gimbal_Yaw_Mech_Error_Kalman, Cloud_Angle_Error[YAW][MECH]);
	//�⻷���
	pTermYaw[OUTER][MECH] = Cloud_Angle_Error[YAW][MECH] * Cloud_Angle_kpid[YAW][MECH][KP];
	pidTermYaw[OUTER][MECH] = pTermYaw[OUTER][MECH];
	pidTermYaw[OUTER][MECH] = constrain_float(pidTermYaw[OUTER][MECH],-PID_Outter_Max,PID_Outter_Max);
	
	//���ٶ����
	Cloud_Palstance_Error[YAW][MECH] = pidTermYaw[OUTER][MECH] - Cloud_Palstance_Measure[YAW][MECH];
	//�ڻ����
	pTermYaw[INNER][MECH]  = Cloud_Palstance_Error[YAW][MECH] * Cloud_Palstance_kpid[YAW][MECH][KP];
	iTermYaw[INNER][MECH] += Cloud_Palstance_Error[YAW][MECH] * Cloud_Palstance_kpid[YAW][MECH][KI] * 0.002f;
	iTermYaw[INNER][MECH]  = constrain_float(iTermYaw[INNER][MECH], -PID_Iterm_Max, PID_Iterm_Max);
	
	pidTermYaw[INNER][MECH] = pTermYaw[INNER][MECH] + iTermYaw[INNER][MECH];
	pidTermYaw[INNER][MECH] = constrain_float(pidTermYaw[INNER][MECH], -PID_Out_Max, PID_Out_Max);
}

/**
  * @brief  yaw������ģʽ
  * @param  void
  * @retval void
  * @attention ����PID�����
  */
void vYaw_Gyro_PositionLoop(void)
{
	//�Ƕ����
	Cloud_Angle_Error[YAW][GYRO] = Cloud_Angle_Target[YAW][GYRO] - Cloud_Angle_Measure[YAW][GYRO];

	//�����п������˲�,������Ƶ�ͷ��ȶ���
	Cloud_Angle_Error[YAW][GYRO] = KalmanFilter(&Gimbal_Yaw_Gyro_Error_Kalman, Cloud_Angle_Error[YAW][GYRO]);

	//�⻷���
	pTermYaw[OUTER][GYRO] = Cloud_Angle_Error[YAW][GYRO] * Cloud_Angle_kpid[YAW][GYRO][KP];
	pidTermYaw[OUTER][GYRO] = pTermYaw[OUTER][GYRO];
	pidTermYaw[OUTER][GYRO] = constrain_float(pidTermYaw[OUTER][GYRO],-PID_Outter_Max,PID_Outter_Max);
	
	//���ٶ����
	Cloud_Palstance_Error[YAW][GYRO] = pidTermYaw[OUTER][GYRO] - Cloud_Palstance_Measure[YAW][GYRO];
	//�ڻ����
	pTermYaw[INNER][GYRO]  = Cloud_Palstance_Error[YAW][GYRO] * Cloud_Palstance_kpid[YAW][GYRO][KP];
	iTermYaw[INNER][GYRO] += Cloud_Palstance_Error[YAW][GYRO] * Cloud_Palstance_kpid[YAW][GYRO][KI] * 0.002f;
	iTermYaw[INNER][GYRO]  = constrain_float(iTermYaw[INNER][GYRO], -PID_Iterm_Max, PID_Iterm_Max);
	
	pidTermYaw[INNER][GYRO] = pTermYaw[INNER][GYRO] + iTermYaw[INNER][GYRO];
	pidTermYaw[INNER][GYRO] = constrain_float(pidTermYaw[INNER][GYRO], -PID_Out_Max, PID_Out_Max)*(-1);
}


