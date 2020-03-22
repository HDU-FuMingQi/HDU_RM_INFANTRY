/*********************************/

//���ļ��жԸ���ģʽ�������������������������
//HDU��д��mysystem.h��

//����С���ݺ͵��̸�����̨Ŀǰ����û�м�����

/********************************/
#include "chassis_task.h"
#include "judge.h"

eChassisAction actChassis=CHASSIS_NORMAL;   //Ĭ�ϵ��̲�������̨����
eChassisCtrlMode  modeChassis=CHASSIS_GYRO_MODE; //Ĭ��Ϊ������ģʽ����

extKalman_t Chassis_Error_Kalman;//����һ��kalmanָ��
PidTypeDef Chassis_Follow_PID;
Chassis_Speed absolute_chassis_speed;
int16_t chassis_motor[4];		//�ĸ�����Ŀ��ת��

const static fp32 nothing[3]={0,0,0};
const static fp32 motorid1_speed_pid[3] ={22,0,12};
const static fp32 motorid2_speed_pid[3] ={22,0, 12};
const static fp32 motorid3_speed_pid[3] ={22,0, 12};
const static fp32 motorid4_speed_pid[3] ={22,0, 12};
const static fp32 Chassis_Follow_pid[3]={0.00001,0,0.00012};


/**
  * @brief  ���̲�����ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
//���̵����޷�
#define iTermChassis_Max             3000     //΢���޷�

	#define		Omni_Speed_Max            9000//7600     //����ˮƽ�ƶ��ٶ��޷�,��ֹ����ģʽ���ٶȳ������ֵ
	#define		STANDARD_MAX_NORMAL       9000//7600     //ƽ�ؿ�������ٶȣ���ֹҡ�˱���*660�������ֵ
	#define		REVOLVE_MAX_NORMAL        9000//7600     //ƽ��Ťͷ����ٶ�
	#define     REVOLVE_KD                (235.f)
	#define     REVOLVE_ANGLE             35

//������ģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Gyro_Chassis_Standard, kKey_Gyro_Chassis_Revolve;//ƽ�ƣ���ת

//��еģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Mech_Chassis_Standard, kKey_Mech_Chassis_Revolve;//ƽ�ƣ���ת

/**************�޷�**************/
#define   LIMIT_CHASSIS_MAX         9000     //������������µ��̵������������

float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//����PID�����������ֵ,���̹�������,���ݵ��ת��ʵʱ�仯
float Chassis_Standard_Move_Max;//����ǰ������ƽ������
float Chassis_Revolve_Move_Max;//����������ת����,���ݲ�ͬ�˶�ģʽʵʱ����,���Բ����ú궨��

float Chassis_Limit_Output_Max=LIMIT_CHASSIS_MAX;//���̹����޷�

void CHASSIS_InitArgument(void)
{
	kKey_Mech_Chassis_Revolve = 40;//���̻�еģʽ��Ťͷ�ٶ���Ӧ����,��̫��,��ȻŤͷ̫��
	kKey_Gyro_Chassis_Revolve = -10;//-8.1;//ע������,����������ģʽ�µ��̸�����̨ת�����ٶ�,���̫��,�����𵴻������
	
	Chassis_Standard_Move_Max  = Omni_Speed_Max;//6800;//9000;//ҡ��ˮƽ�ƶ��޷�
	Chassis_Revolve_Move_Max   = Omni_Speed_Max;//6800;//7000;//ҡ������Ťͷ�޷�,������΢��һ��,������̨Ť̫��ᵼ����̨ײ����λ,̫���ֻᵼ�µ�Ŀ��λ�ú�ζ�
	
	KalmanCreate(&Chassis_Error_Kalman, 1, 0);      //���ڱջ���ʹ�ã����ڻ���̫���

}


void ChassisFun(void const * argument)
{
	portTickType currentTime;
	Chassis_Follow_PID.angle_max=8192;
	Chassis_Follow_PID.angle_min=0;
	Chassis_Follow_PID.Dead_Zone=100;
	Chassis_Follow_PID.I_Separation=1e30;
	Chassis_Follow_PID.gama=0.2;
	PID_Init(&Chassis_Follow_PID,PID_POSITION,Chassis_Follow_pid,0.015,0.003);//���̸�����̨PID

	
	/******************���̵��PID*****************************************/
	Motor_Init(&Chassis_Motor1,1,nothing,0,0,motorid1_speed_pid,20000,10000);//
	Motor_Init(&Chassis_Motor2,2,nothing,0,0,motorid2_speed_pid,20000,10000);
	Motor_Init(&Chassis_Motor3,3,nothing,0,0,motorid3_speed_pid,20000,10000);
	Motor_Init(&Chassis_Motor4,4,nothing,0,0,motorid4_speed_pid,20000,10000);
	while(1)
	{
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		GetEnvironmentChassisMode();	//���ó�����������
		LimitChassisMotorCurrent();		//��������������������
		/***********************ѡ�����ģʽΪң����/���̽��п���***************/
		switch(ControlMode)
		{
			case REMOTE:
				RemoteControlChassis();
				break;
			case KEYBOARD:
				KeyboardControlChassis();
				break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}
/**
  * @brief  ���õ��̵������������ֵ
  * @param  ���1���ֵ�����2���ֵ�����3���ֵ�����4���ֵ
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
  * @brief  ���ݲ�ͬ�����������
  * @param  void
  * @retval void
  * @attention 
  */
void LimitChassisMotorCurrent(void)
{
	switch(actChassis)
	{
		case CHASSIS_NORMAL:		//���̲�������̨
			switch(EnvironmentMode)
			{
				case NOMAL:			//��ͨ����
					SetChassisMotorMaxCurrent(NOMOAL_CHASSIS_MAX1,NOMOAL_CHASSIS_MAX2,NOMOAL_CHASSIS_MAX3,NOMOAL_CHASSIS_MAX4);
					break;
				case CLIMBING:		//���µ���
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_MAX1,CLIMBING_CHASSIS_MAX2,CLIMBING_CHASSIS_MAX3,CLIMBING_CHASSIS_MAX4);
					break;
			}
			break;

		case CHASSIS_PISA:				//45��ģʽ������̸�����̨������ͬ
			switch(EnvironmentMode)
			{
				case NOMAL:			//��ͨ����
					SetChassisMotorMaxCurrent(NOMAL_FOLLOW_CHASSIS_MAX1,NOMAL_FOLLOW_CHASSIS_MAX2,NOMAL_FOLLOW_CHASSIS_MAX3,NOMAL_FOLLOW_CHASSIS_MAX4);
					break;
				case CLIMBING:		//���µ���
					SetChassisMotorMaxCurrent(CLIMBING_FOLLOW_CHASSIS_MAX1,CLIMBING_FOLLOW_CHASSIS_MAX2,CLIMBING_FOLLOW_CHASSIS_MAX3,CLIMBING_FOLLOW_CHASSIS_MAX4);
					break;
			}
			break;

		case CHASSIS_CORGI:				//Ťƨ��ģʽ����С����������ͬ
			switch(EnvironmentMode)
			{
				case NOMAL:			//��ͨ����
						SetChassisMotorMaxCurrent(NOMAL_GYRO_CHASSIS_MAX1,NOMAL_GYRO_CHASSIS_MAX2,NOMAL_GYRO_CHASSIS_MAX3,NOMAL_GYRO_CHASSIS_MAX4);
						break;
				case CLIMBING:		//���µ���
					SetChassisMotorMaxCurrent(CLIMBING_GYRO_CHASSIS_MAX1,CLIMBING_GYRO_CHASSIS_MAX2,CLIMBING_GYRO_CHASSIS_MAX3,CLIMBING_GYRO_CHASSIS_MAX4);
					break;
			}
			break;
			
		case  CHASSIS_SZUPUP:  //�ֶ�����ģʽ
			SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_SZUPUP_MAX1,CLIMBING_CHASSIS_SZUPUP_MAX2,CLIMBING_CHASSIS_SZUPUP_MAX3,CLIMBING_CHASSIS_SZUPUP_MAX4);
			break;
		
		case  CHASSIS_MISS:    //�Զ�����ģʽ
			switch(EnvironmentMode)
			{
				case NOMAL:			//��ͨ����
						SetChassisMotorMaxCurrent(NOMAL_CHASSIS_MISS_MAX1,NOMAL_CHASSIS_MISS_MAX2,NOMAL_CHASSIS_MISS_MAX3,NOMAL_CHASSIS_MISS_MAX4);
						break;
				case CLIMBING:		//���µ���
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_MISS_MAX1,CLIMBING_CHASSIS_MISS_MAX2,CLIMBING_CHASSIS_MISS_MAX3,CLIMBING_CHASSIS_MISS_MAX4);
					break;
			}
			break;
			
		case CHASSIS_GYROSCOPE:			//С����ģʽ
			switch(EnvironmentMode)
			{
				case NOMAL:			//��ͨ����
						SetChassisMotorMaxCurrent(NOMAL_CHASSIS_GYROSCOPE_MAX1,NOMAL_CHASSIS_GYROSCOPE_MAX2,NOMAL_CHASSIS_GYROSCOPE_MAX3,NOMAL_CHASSIS_GYROSCOPE_MAX4);
						break;
				case CLIMBING:		//���µ���
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_GYROSCOPE_MAX1,CLIMBING_CHASSIS_GYROSCOPE_MAX2,CLIMBING_CHASSIS_GYROSCOPE_MAX3,CLIMBING_CHASSIS_GYROSCOPE_MAX4);
					break;
			}
			break;
			
		case CHASSIS_FOLLOW_GIMBAL:			//���̸�����̨
			switch(EnvironmentMode)
			{
				case NOMAL:			//��ͨ����
						SetChassisMotorMaxCurrent(NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX1,NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX2,NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX3,NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX4);
						break;
				case CLIMBING:		//���µ���
					SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX1,CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX2,CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX3,CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX4);
					break;
			}
			break;
		
		default:
			break;
			
	}
}
/**
  * @brief  ң�������Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention 
  */



void RemoteControlChassis(void)
{
	/***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
	switch(actChassis)
	{
		case CHASSIS_FOLLOW_GIMBAL://������̨
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY;
			absolute_chassis_speed.vw=PID_Calc(&Chassis_Follow_PID,FindMinAnglePNY(),0);//PIDʹ���̸�����̨�ٶ�
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//������������Ŀ���ٶ�
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//���ø��������Ŀ���ٶ�
			break;
		case CHASSIS_NORMAL://��������̨
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY;
			absolute_chassis_speed.vw=0;
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//������������Ŀ���ٶ�
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//���ø��������Ŀ���ٶ�
			break;
		case CHASSIS_PISA:		//45��ģʽ
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY;
			absolute_chassis_speed.vw=PID_Calc(&Chassis_Follow_PID,FindMinAngleFortyFive(),0);//PIDʹ���̸�����̨�ٶ�
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//������������Ŀ���ٶ�
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//���ø��������Ŀ���ٶ�
			break;
		case CHASSIS_GYROSCOPE:		//С����ģʽ
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY;
			absolute_chassis_speed.vw=0.006;
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//������������Ŀ���ٶ�
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//���ø��������Ŀ���ٶ�
			break;
		case CHASSIS_CORGI:		//Ťƨ��ģʽ
			absolute_chassis_speed.vx=(float)rc.ch1/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX;
			absolute_chassis_speed.vy=(float)rc.ch2/SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY;
			absolute_chassis_speed.vw=PID_Calc(&Chassis_Follow_PID,FindMinAngleFortyFive(),0);//PIDʹ���̸�����̨�ٶ�
			Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//������������Ŀ���ٶ�
			Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//���ø��������Ŀ���ٶ�
			break;
		default:
			break;
	}
	/************************************���̵���ٶȻ�����*********************************************/
			PID_Calc(&Chassis_Motor1.Motor_PID_Speed,Chassis_Motor1.motor_value->speed_rpm,
				Chassis_Motor1.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor2.Motor_PID_Speed,Chassis_Motor2.motor_value->speed_rpm,
							Chassis_Motor2.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor3.Motor_PID_Speed,Chassis_Motor3.motor_value->speed_rpm,
							Chassis_Motor3.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor4.Motor_PID_Speed,Chassis_Motor4.motor_value->speed_rpm,
							Chassis_Motor4.motor_value->target_speed_rpm);
	/************************************�������������͸����*********************************************/
		Chassis_Power_Limit();//��������,�������·���
		set_moto1234_current(&hcan1,Chassis_Motor1.Motor_PID_Speed.out,Chassis_Motor2.Motor_PID_Speed.out
									,Chassis_Motor3.Motor_PID_Speed.out,Chassis_Motor4.Motor_PID_Speed.out);
}

/*****************���̹���*************************/

#define  CHAS_CURRENT_LIMIT        40000    //�ĸ����ӵ��ٶ��ܺ����ֵ,�������*4,�޹��ʵ���������

float fChasCurrentLimit = CHAS_CURRENT_LIMIT;//����4�����ӵ��ٶ��ܺ�
float fTotalCurrentLimit;//��������,ƽ��ģʽ�·����Ǿ��ȵ�
float WARNING_REMAIN_POWER = 60;//����ϵͳʣ�ཹ���������������ֵ��ʼ�޹���,40Ťƨ�ɻᳬ����,ƽ�ؿ����ᳬ

/**
  * @brief  ���̹�������
  * @param  void
  * @retval void
  * @attention  �ڵ��������������,��Ҫ�Ǳ������㷨,ICRA
  */
void Chassis_Power_Limit(void)
{
	/*********************�洫�㷨*************************/
	float    kLimit = 0;//��������ϵ��
	float    chassis_totaloutput = 0;//ͳ�����������
	float    Joule_Residue = 0;//ʣ�ཹ����������
	int16_t  judgDataCorrect = 0;//����ϵͳ�����Ƿ����	
	static int32_t judgDataError_Time = 0;
	
	
	judgDataCorrect = JUDGE_sGetDataState();//����ϵͳ�����Ƿ����
	Joule_Residue = JUDGE_fGetRemainEnergy();//ʣ�ཹ������	
	
	//ͳ�Ƶ��������
	chassis_totaloutput=abs_float(Chassis_Motor1.Motor_PID_Speed.out)+abs_float(Chassis_Motor2.Motor_PID_Speed.out)
														+abs_float(Chassis_Motor3.Motor_PID_Speed.out)+abs_float(Chassis_Motor4.Motor_PID_Speed.out);
	if(judgDataCorrect == JUDGE_DATA_ERROR)//����ϵͳ��Чʱǿ������
	{
		judgDataError_Time++;
		if(judgDataError_Time > 100)
		{
			fTotalCurrentLimit = 10000;//��Ϊ����1/4
		}
	}
	else
	{
		judgDataError_Time = 0;
		//ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
		if(Joule_Residue < WARNING_REMAIN_POWER)
		{
			kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
						* (float)(Joule_Residue / WARNING_REMAIN_POWER);
			
			fTotalCurrentLimit = kLimit * fChasCurrentLimit;
		}
		else   //���������ָ���һ����ֵ
		{
			fTotalCurrentLimit = fChasCurrentLimit;
		}
	}
	
	//���̸�����������·���
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Motor1.Motor_PID_Speed.out = (float)(Chassis_Motor1.Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);	
	}
}

/**
  * @brief  ���̿��Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention 
  */

#define Omni_SupCap_Max				 10000		//���ݷŵ����������ٶ�
#define 	Omni_Speed_Max            9000     //����ˮƽ�ƶ��ٶ��޷�,��ֹ����ģʽ���ٶȳ������ֵ


void KeyboardControlChassis(void)
{
	float speed_max;
	if(Cap_Out_Can_Open() == TRUE)//���ݷŵ�
	{
		speed_max = Omni_SupCap_Max;
	}
	else
	{
		speed_max = Omni_Speed_Max;
	}
	
	CHAS_Key_Ctrl();
	Absolute_Cal(&absolute_chassis_speed,(float)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//������������Ŀ���ٶ�
	Mecanum_Set_Motor_Speed(chassis_motor,Chassis_Motor1.motor_value);//���ø��������Ŀ���ٶ�
	
	/************************************���̵���ٶȻ�����*********************************************/
			PID_Calc(&Chassis_Motor1.Motor_PID_Speed,Chassis_Motor1.motor_value->speed_rpm,
				Chassis_Motor1.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor2.Motor_PID_Speed,Chassis_Motor2.motor_value->speed_rpm,
							Chassis_Motor2.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor3.Motor_PID_Speed,Chassis_Motor3.motor_value->speed_rpm,
							Chassis_Motor3.motor_value->target_speed_rpm);
			PID_Calc(&Chassis_Motor4.Motor_PID_Speed,Chassis_Motor4.motor_value->speed_rpm,
							Chassis_Motor4.motor_value->target_speed_rpm);
	/************************************�������������͸����*********************************************/
		Chassis_Motor1.Motor_PID_Speed.out = constrain_float(Chassis_Motor1.Motor_PID_Speed.out ,-speed_max, speed_max);
		Chassis_Motor2.Motor_PID_Speed.out = constrain_float(Chassis_Motor2.Motor_PID_Speed.out ,-speed_max, speed_max);
		Chassis_Motor3.Motor_PID_Speed.out = constrain_float(Chassis_Motor3.Motor_PID_Speed.out ,-speed_max, speed_max);
		Chassis_Motor4.Motor_PID_Speed.out = constrain_float(Chassis_Motor4.Motor_PID_Speed.out ,-speed_max, speed_max);
		if(actChassis == CHASSIS_SZUPUP)//����ǰ�ֻ�򻬣������޵ñȺ���ҪС,,�˴���ô����ʵ����˵
		{
			Chassis_Motor1.Motor_PID_Speed.out = constrain_float(Chassis_Motor1.Motor_PID_Speed.out ,-speed_max, speed_max);
			Chassis_Motor2.Motor_PID_Speed.out = constrain_float(Chassis_Motor2.Motor_PID_Speed.out ,-speed_max, speed_max);
			Chassis_Motor3.Motor_PID_Speed.out = constrain_float(Chassis_Motor3.Motor_PID_Speed.out ,-Omni_Speed_Max, Omni_Speed_Max);
			Chassis_Motor4.Motor_PID_Speed.out = constrain_float(Chassis_Motor4.Motor_PID_Speed.out ,-Omni_Speed_Max, Omni_Speed_Max);
		}
		Chassis_Power_Limit();//��������,�������·���
		set_moto1234_current(&hcan1,Chassis_Motor1.Motor_PID_Speed.out,Chassis_Motor2.Motor_PID_Speed.out
									,Chassis_Motor3.Motor_PID_Speed.out,Chassis_Motor4.Motor_PID_Speed.out);
}
/**
  * @brief  ȷ�����κ͵���״̬
  * @param  void
  * @retval void
  * @attention ͨ��ң����/����
  */
void GetEnvironmentChassisMode(void)
{
	switch(ControlMode)
	{
		case REMOTE:
			//ѡ�񳵵���ģʽ
			if(rc.sw1==1)
				actChassis=CHASSIS_NORMAL;	//���̲�������̨

			else if(rc.sw1==3)
				actChassis=CHASSIS_FOLLOW_GIMBAL;	//���̸�����̨
			else if(rc.sw1==2)	
				actChassis=CHASSIS_GYROSCOPE;		//С����ģʽ
			//ѡ����������
			break;
	
		case KEYBOARD:    //����ģʽ
			/*------------------��ͨģʽ,����ģʽ�л��ж�-------------------*/	
			Chassis_NORMAL_Mode_Ctrl();								
			break;
		
		default:
			break;
	}
}

/**
  * @brief  ���ֽ���
  * @param  
  * @retval 
  * @attention ת��Ϊ�ٶ�Motor[i].target_speed_rpm
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
 * @param absolute_speed ����������Ҫ���ٶ�
 * @param angle ��̨����ڵ��̵ĽǶ�
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
  * @brief  �ҳ���+-y����Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
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
  * @brief  �ҳ���45������Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
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
  * @brief  ��������״̬
  * @param  void
  * @retval void
  * @attention ״ֵ̬0
  */

float Slope_Chassis_Move_Z;//б�¼�������ƶ�����,����Ŀ���������ʵʱб��ֵ

void CHASSIS_REST(void)
{
	Slope_Chassis_Move_Z = 0;//Ťƨ��ʵʱ���б��
	absolute_chassis_speed.vx  = 0;
	absolute_chassis_speed.vy  = 0;
	absolute_chassis_speed.vw  = 0;
}



/***********************************************************************************************
     *  ����Ĳ����Ǽ���ģʽ
			
				


 ****************************************************************************************************/

/***********���̸���ģʽ��һЩ��������*************/
/**
  * @brief  ���̿��Ƶ����ƶ�
  * @param  void
  * @retval void
  * @attention ģʽѡ��,����ĳģʽ��ǵ�д�˳�����ͨģʽ���ж�
  * �ް������»�һֱ�����Զ�����ģʽ,����ģʽ�л���İ�����������ģʽ�л�ѡ��ģʽ
  */

//ÿ2msִ��һ����������������ʱ
uint8_t remot_change = TRUE;

//Ťƨ��
bool Chass_Switch_F = 1;
u8 	 Chass_Key_F_Change = 0;

//45��
bool Chass_Switch_X = 1;
u8 	 Chass_Key_X_Change = 0;

//С����
bool Chass_Switch_G = 1;
u8 	 Chass_Key_G_Change = 0;

//��ͬģʽ��,б�º�����Ӧ��ʱ��ֵ,һ������ͨ�ٶȾ���
#define    TIME_INC_NORMAL           10//6//4	  //����б��,Խ�������ٶ�Խ��,���ʱ��Խ��
#define    TIME_DEC_NORMAL           500//180        //����б��,Խ���С�ٶ�Խ��(һ��Ҫ��INC��һ��,�����ɿ������ܸ���Ϊ0,̫�������ɵ���ͣ������ʱ����Ծ)

#define    REVOLVE_SLOPE_CORGI       50       //����Ťƨ��ģʽб��,Խ��Խ��,���ʱ��Խ��


//��ͬģʽ�µ�����ٶ�
#define    REVOLVE_MAX_CORGI         9000//5000     //����Ťƨ������ٶ�,̫����ýǶȹ���

void CHAS_Key_Ctrl(void)
{
	if(remot_change == TRUE)//�մ�ң��ģʽ�й���,Ĭ��Ϊ������ģʽ
	{
		modeChassis = CHASSIS_GYRO_MODE;
		remot_change = FALSE;
	}
	
	if(GIMBAL_IfBuffHit() == TRUE)//���ģʽΪ��е
	{
		modeChassis = CHASSIS_MECH_MODE;
	}
	
	switch (actChassis)      //SB keil �о���,����ģʽѡ�񣬲�����������ͨģʽ
	{
		/*------------------Ťƨ��ģʽ-------------------*/			
		case CHASSIS_CORGI:	
			if(!IF_KEY_PRESSED_F)//F�ɿ�
			{
				Chass_Switch_F = 1;
			}
			
			if(IF_KEY_PRESSED_F && Chass_Switch_F == 1)
			{
				Chass_Switch_F = 0;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
			}
			//����ǰ���ƶ�,��������ƽ�ơ�QE���л���еģʽ�˳�
			if(Chass_Key_F_Change)
			{
				modeChassis = CHASSIS_GYRO_MODE;//������ģʽ,���̸�����̨��
				
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);		
			}
			else
			{
				actChassis = CHASSIS_NORMAL;//�˳�Ťƨ��ģʽ
			}
			
//			//���ֿ���ʱҪ�ر�Ť��
//			if((IF_KEY_PRESSED_CTRL || GIMBAL_IfGIMBAL_LEVEL()==TRUE ) && !IF_KEY_PRESSED_F)
//			{
//				Chass_Switch_F = 1;
//				Chass_Key_F_Change ++;
//				Chass_Key_F_Change %= 2;
//				actChassis = CHASSIS_NORMAL;//�˳�Ťƨ��ģʽ
//			}
//			else if(IF_KEY_PRESSED_X)//����ʱ����45��ģʽ
//			{
//				Chass_Switch_F = 1;
//				Chass_Key_F_Change ++;
//				Chass_Key_F_Change %= 2;
//				actChassis = CHASSIS_PISA;
//			}			
//		break;
//			
		/*-------------���ģʽ-------------*/
		case CHASSIS_ROSHAN:
						
			if(GIMBAL_IfBuffHit( ) != TRUE)
			{
				actChassis = CHASSIS_NORMAL;//�˳����ģʽ
				modeChassis = CHASSIS_GYRO_MODE;//�л�������ģʽ
			}	
			else
			{
				modeChassis = CHASSIS_MECH_MODE;//������̽����еģʽ
				CHASSIS_REST();//Ŀ���ٶ���0
			}				
		break;
				
//		/*---------------����ģʽ,���̵���----------------*/
//		case CHASSIS_SLOW:
//			if (Magazine_IfOpen() != TRUE)//���ֹر�
//			{
//				actChassis = CHASSIS_NORMAL;//�����˳�����ģʽ
//			}
//			else
//			{
//				modeChassis = CHASSIS_MECH_MODE;//����ʱ���̽����еģʽ
//			
//				Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SLOW, TIME_INC_SLOW );
//				Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SLOW );
//			}
//		break;
//				
//		/*-------------�ֶ�����ģʽ-------------*/
		case CHASSIS_SZUPUP:
			CHASSIS_SZUPUP_Mode_Ctrl();
		break;
			
//		/*-------------�Զ�����ģʽ-------------*/
		case CHASSIS_MISS:
			CHASSIS_MISS_Mode_Ctrl();
		break;
		
//		/*-------------45��Ե�ģʽ-------------*/
		case CHASSIS_PISA:
			if(!IF_KEY_PRESSED_X)//F�ɿ�
			{
				Chass_Switch_X = 1;
			}
			
			if(IF_KEY_PRESSED_X && Chass_Switch_X == 1)
			{
				Chass_Switch_X = 0;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
			}
			
			if(Chass_Key_X_Change)//ǰ���˳�
			{
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_PISA_Mode_Ctrl();
			}
			else
			{
				actChassis = CHASSIS_NORMAL;
			}
			
			//���ֿ���ʱҪ�ر�
//			if( (IF_KEY_PRESSED_CTRL 
//					|| GIMBAL_IfGIMBAL_LEVEL()==TRUE 
//						|| IF_KEY_PRESSED_W || IF_KEY_PRESSED_S) 
//							&& !IF_KEY_PRESSED_X)
//			{
//				Chass_Switch_X = 1;
//				Chass_Key_X_Change ++;
//				Chass_Key_X_Change %= 2;
//				actChassis = CHASSIS_NORMAL;//�˳�Ťƨ��ģʽ
//			}
//			else if(IF_KEY_PRESSED_F)//����ʱ����Ť��ģʽ
//			{
//				Chass_Switch_X = 1;
//				Chass_Key_X_Change ++;
//				Chass_Key_X_Change %= 2;
//				actChassis = CHASSIS_CORGI;
//			}
		break;
		case CHASSIS_GYROSCOPE:
			if(!IF_KEY_PRESSED_G)//G�ɿ�
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

/*************************���̼���ģʽ����ģʽС����****************************/

/**************б��***************/


uint16_t timeInc;//б�����ӱ仯ʱ��
uint16_t timeInc_Saltation;//ǰ����ͻ���µ�б��������,�����������ҪС
int16_t   timeXFron,    timeXBack,    timeYLeft,    timeYRigh;//����  s  w  d   a

#define    TIME_INC_SLOW             1		  //����ģʽ���ٶȱ仯����
#define    TIME_INC_SZUPUP           3		  //�ֶ�����ģʽ���ٶȱ仯����

#define    TIME_INC_SALTATION        1        //ͻȻ����������ٶȱ仯����

#define    REVOLVE_SLOPE_NORMAL      80       //������ͨģʽб��,Խ��Խ��,���ʱ��Խ��
#define    REVOLVE_SLOPE_CORGI       50       //����Ťƨ��ģʽб��,Խ��Խ��,���ʱ��Խ��

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//����ģʽ��Ťͷб��,��Ҫ����Ťƨ��ģʽ��
float Slope_Chassis_Revolve_Move;

/**
  * @brief  ����ģʽ�µ����˶�����
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  ���̿���ǰ������ƽ��,ƽ���޻�е��������ģʽ֮��
  *             ��Ҫ��ȡʱ��������б�º�������
  */
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp )
{
	static portTickType  ulCurrentTime = 0;
	static uint32_t  ulDelay = 0;
	float k_rc_z = 1;//����Z�ٶȵ���ǰ������ƽ�����ٱ�
	
	Chassis_Standard_Move_Max = sMoveMax;//�����ٶ��޷�,ˮƽ�ƶ�
	timeInc      = sMoveRamp;
	
	ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
	
	if(fabs(absolute_chassis_speed.vw) > 800)//Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
	{
		k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(absolute_chassis_speed.vw) + 800) * (Chassis_Revolve_Move_Max - fabs(absolute_chassis_speed.vw) + 800) )
					/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
		
		LimtValue_f(&k_rc_z,0,1);
	}
	else
	{
		k_rc_z = 1;
	}
	
	if (ulCurrentTime >= ulDelay)//ÿ10ms�仯һ��б����
	{
		ulDelay = ulCurrentTime + TIME_STAMP_10MS;


		if(actChassis == CHASSIS_NORMAL && !KEY_PRESSED_OFFSET_SHIFT)//ֻ��һ��ģʽ�²��ж��ٶ�ͻ�����,��ֹ��
		{
			if (IF_KEY_PRESSED_W)//�ȳ������ݳ����ٲ��Ե��ݷŵ�ʱ�Ƿ�Ҫȫ������
			{
				timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
				//ǰ��X������,���ڴ˼��볬�����ݰ����ж�
				if( absolute_chassis_speed.vx < sMoveMax/2.5 )//ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{//�����ٶ��Ƿ�����ٶȵ�1/5���жϲ�֪���Ƿ����
					timeInc_Saltation = TIME_INC_SALTATION;//������ģʽ�����ٶȷ���ͻ��
				}
				else			//�Ѿ����˴�ʱ������������һ�����ٶ�
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//ͬ��
				//����X�Ǹ�
				if( absolute_chassis_speed.vx  > (-sMoveMax)/2.5 )//ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{
					timeInc_Saltation = TIME_INC_SALTATION;//������ģʽ�����ٶȷ���ͻ��
				}
				else			//�Ѿ����˴�ʱ������������һ�����ٶ�
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
		
			//����ģʽ��ȫ���ƶ�,б��������,ע������,��������*б�±����õ��������ӵ�ֵ,ģ��ҡ��
			//ǰ�������б���Ǳ仯��
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc_Saltation, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc_Saltation, TIME_DEC_NORMAL ) );

			//���ҵ�����б�¸�ǰ��һ��,����
			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc/1.5, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc/1.5, TIME_DEC_NORMAL ) );


			absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//ǰ�����
			absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//���Ҽ���
		}
		else		//����ģʽ����Ҫ�����ٶȷ���ͻ�����⴦��
		{
			if (IF_KEY_PRESSED_W)
			{
				timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//ͬ��
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
				absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//ǰ�����
				absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//���Ҽ���
			}
		}
	}
}

/**
  * @brief  �����Ƶ�����ת,����QEC���ƿ���תȦ
  * @param  �ٶ��������� 
  * @retval void
  * @attention  ������������ת
  */
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax )
{
//	static int16_t sErrorPrev = 0;//�ϴ�ƫ�����
	float sErrorReal =IMU_Vale.Yaw-GIMBAL_YAW_ENCODER_MIDDLE1;//yawƫ���������
	
	Chassis_Revolve_Move_Max = sRevolMax;//������ת����
	
	if(modeChassis == CHASSIS_GYRO_MODE)//������ģʽ
	{
		AngleLoop(&sErrorReal,8192);//��ȡʵʱƫ��,����Ťƨ���µĵ���λ�ò���
		absolute_chassis_speed.vw = PID_Calc(&Chassis_Follow_PID,sErrorReal,kKey_Gyro_Chassis_Revolve);
	}
	else     //��еģʽ
	{
		absolute_chassis_speed.vw = constrain_float( MOUSE_X_MOVE_SPEED*kKey_Mech_Chassis_Revolve, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);
	}
}



/**************����ģʽ��������********************/

/**
  * @brief  ���̼���б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
	float  factor = 0;

	
	factor = 0.15 * sqrt( 0.15 * (*time) );  //�����ٶ�б��,time�ۼӵ�296.3б�¾����
	
	if (status == 1)//����������
	{
		if (factor < 1)//��ֹtime̫��
		{
			*time += inc;
		}
	}
	else		//�����ɿ�
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

	LimtValue_f( &factor, 0, 1 );//ע��һ����float�����޷�

	return factor;  //ע�ⷽ��
}

/**
  * @brief  ���̼���ģʽѡ��,������Ӧ
  * @param  void
  * @retval void
  * @attention ���̼��̿���״̬�µ�����ģʽ�л�������
  */


void Chassis_NORMAL_Mode_Ctrl(void)
{
	if(!IF_KEY_PRESSED_F)//F�ɿ�
	{
		Chass_Switch_F = 1;
	}
	
	if(!IF_KEY_PRESSED_X)//X�ɿ�
	{
		Chass_Switch_X = 1;
	}
	
	if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_F == 1)//F����,�л���Ťƨ��(����һֱ��F)
	{
		Chass_Switch_F = 0;
		Chass_Key_F_Change ++;
		Chass_Key_F_Change %= 2;
		actChassis = CHASSIS_CORGI;//�ǵ�д�����˳�Ťƨ��ģʽ�ĺ���
	}
//	else if(Magazine_IfOpen() == TRUE)//���ֿ���,���벹��ģʽ
//	{
//		actChassis = CHASSIS_SLOW;
//	}
	else if(IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL)//W Ctrlһ�𰴽�������ģʽ
	{
		actChassis = CHASSIS_SZUPUP;//����ģʽ
	}
	else if(JUDGE_IfArmorHurt() == TRUE && GIMBAL_IfBuffHit() == FALSE)//ֻҪ�˺��������ް�������,�ͻ�����Զ�����ģʽ
	{
		actChassis = CHASSIS_MISS;//���ʱ�ر��Զ�����
	}
	else if(GIMBAL_IfBuffHit() == TRUE)//���ģʽ
	{
		actChassis = CHASSIS_ROSHAN;
	}
	else if (IF_KEY_PRESSED_X && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_X == 1)//x����,�л���45��
	{
		Chass_Switch_X = 0;
		Chass_Key_X_Change ++;
		Chass_Key_X_Change %= 2;
		actChassis = CHASSIS_PISA;
	}
	else 					
	{		
		//���ʱǿ�ƽ����еģʽ
		if(IF_KEY_PRESSED_CTRL || GIMBAL_IfBuffHit() == TRUE)      //��סCTRL�����еģʽ
		{
			modeChassis = CHASSIS_MECH_MODE;
		}
		else			//�ɿ�CTRL����������ģʽ
		{
			modeChassis = CHASSIS_GYRO_MODE;
		}

		//�ƶ��ٶȿ���
//		if(Cap_Out_Can_Open() == TRUE)//���ݷŵ�
//		{
//			Chassis_Keyboard_Move_Calculate(Omni_SupCap_Max, TIME_INC_NORMAL);//�ŵ�ʱ����ٶ�Ҫ��
//			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);
//		}
//		else
//		{
//			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);//�����ٶ����ֵ��б��ʱ��
//			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);			
//		}
		actChassis=CHASSIS_FOLLOW_GIMBAL;
	}	
}
	


/**************����ģʽ����********************/

/**
  * @brief  Ťƨ��ģʽ(λ�ò����)
  * @param  �ٶ���������    ���ӵ����������ʱ��
  * @retval void
  * @attention  ����ʱ�䣬Ť��λ�˾ͻ���   //*******Ŧ�ĽǶ�Ӧ���ǲ��Ե�*****************/
  
//Ťƨ�ɻ���ѡ��
#define    CORGI_BEGIN    0    
#define    CORGI_LEFT     1
#define    CORGI_RIGH     2

uint16_t  stateCorgi = CORGI_BEGIN;//�������Ť,Ĭ�ϲ�Ť
bool    IfCorgiChange = FALSE;//�Ƿ�Ť������һ��
int16_t  corgi_angle_target = 0;//����Ŀ��Ƕ�


void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp)
{
		int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;


	Chassis_Revolve_Move_Max = sRevolMax;//����ٶ�����
	Slope_Chassis_Revolve_Move = sRevolRamp;//Ťͷб������

	sAngleError = FindMinAnglePNY();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	absolute_chassis_speed.vx = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	absolute_chassis_speed.vy = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	//�ؼ�:��������......
	switch (stateCorgi)
	{
		case CORGI_BEGIN:	//�Ժ���������ø����(��־λ��ͣȡ��),���ÿ�ʼŤͷ�ķ������	  
			corgi_angle_target = -900;//�ɸ�����ƶ��Ƕ�,�Զ�����ģʽ�±�����ʱ��Ť���Ƕ�
			IfCorgiChange = FALSE;
			stateCorgi    = CORGI_LEFT;
		break;
		
		case CORGI_LEFT:
			corgi_angle_target = -1024;//�ɸ�����ƶ��Ƕ�			
			IfCorgiChange = FALSE;

			if (sAngleError < -700)//�Ƕ�������700
			{
					stateCorgi = CORGI_RIGH;
				  IfCorgiChange = TRUE;//��ǿ��Ի���
			}			
		break;
			
		case CORGI_RIGH:		
			corgi_angle_target = 1024;
			IfCorgiChange = FALSE;

			if (sAngleError > 700)//�Ƕ�������700
			{
				stateCorgi = CORGI_LEFT;
				IfCorgiChange = TRUE;//��ǿ��Ի���
			}			
		break;
	}

	absolute_chassis_speed.vw = PID_Calc( &Chassis_Follow_PID, (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}


/**
  * @brief  �ֶ�����ģʽ
  * @param  void
  * @retval void
  * @attention  
  */

#define    STANDARD_MAX_SZUPUP       3000//5000//6000//4000//3600	  //�ֶ�����ģʽ��ˮƽ�ƶ��ٶ�
#define    REVOLVE_MAX_SZUPUP        9000     //�ֶ�����ģʽ��Ťͷ�ٶ�

void CHASSIS_SZUPUP_Mode_Ctrl(void)
{
	if( !IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL)//�ɿ�����һ���˳�����ģʽ
	{
		actChassis = CHASSIS_NORMAL;//�����˳�����ģʽ
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;//������ģʽ
		
		Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SZUPUP, TIME_INC_SZUPUP );
		Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SZUPUP );
	}
}

/**
  * @brief  �Զ�����ģʽ
  * @param  void
  * @retval void
  * @attention  
  */

//�Զ�����
#define   MISS_MAX_TIME    1000    //�Զ���������λʱ��,��λ2*ms
uint32_t  Miss_Mode_Time = 0;//�Զ������ѹ�ʱ��

void CHASSIS_MISS_Mode_Ctrl(void)
{
	int16_t  sAngleError   = 0;
//	sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//�а������»��߳�ʱ��û���ܵ��������˳��Զ�����,��֤��������
	if( IF_KEY_PRESSED || Miss_Mode_Time > MISS_MAX_TIME )
	{
		actChassis = CHASSIS_NORMAL;//�����л�������ģʽ
		Miss_Mode_Time = 0;
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;
		
		if(JUDGE_IfArmorHurt() == TRUE 	//װ�װ����ݸ���,���ܵ��µ��˺�
			|| IfCorgiChange == FALSE)  //ƨ��û��Ť���Ա�
		{
			//����Ťƨ��һ��
			CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
			Miss_Mode_Time = 0;
		}
		else
		{
			Slope_Chassis_Move_Z = 0;//Ťƨ��ʵʱ���б��
			absolute_chassis_speed.vx = 0;
			absolute_chassis_speed.vy = 0;
			absolute_chassis_speed.vw = PID_Calc( &Chassis_Follow_PID,(sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
			Miss_Mode_Time++;
		}	
	}
}


/**
  * @brief  45��ģʽ
  * @param  void
  * @retval void
  * @attention  //�ɸ�����ƶ��Ƕ�    ���ֵ���п����Ǵ���ĵ���ת����45��
  */

//������ģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Gyro_Chassis_Standard, kKey_Gyro_Chassis_Revolve;//ƽ�ƣ���ת

void CHASSIS_PISA_Mode_Ctrl(void)
{
	int16_t  corgi_angle_target = 0;//����Ŀ��Ƕ�
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;

	sAngleError = 	FindMinAnglePNY();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	absolute_chassis_speed.vx  = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	absolute_chassis_speed.vy  = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	corgi_angle_target = -1024;//�ɸ�����ƶ��Ƕ�

	absolute_chassis_speed.vw  = PID_Calc( &Chassis_Follow_PID, (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}

/**
  * @brief  С����ģʽ
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  
  */
void CHASSIS_GYROSCOPE_Mode_Ctrl(int16_t sMoveMax,int16_t sMoveRamp)
{
//	static portTickType  ulCurrentTime = 0;
//	static uint32_t  ulDelay = 0;
//	float k_rc_z = 1;//����Z�ٶȵ���ǰ������ƽ�����ٱ�

//	Chassis_Standard_Move_Max = sMoveMax;//�����ٶ��޷�,ˮƽ�ƶ�
//	timeInc      = sMoveRamp;
//	
//	ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
//	
//	
//	if (ulCurrentTime >= ulDelay)//ÿ10ms�仯һ��б����
//	{
//		ulDelay = ulCurrentTime + TIME_STAMP_10MS;		//����ģʽ����Ҫ�����ٶȷ���ͻ�����⴦��
//		if (IF_KEY_PRESSED_W)
//		{
//			timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
//		}

//		if (IF_KEY_PRESSED_S)
//		{
//			timeXFron = 0;//ͬ��
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
//			absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//ǰ�����
//			absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//���Ҽ���
//		}
//	}
//	absolute_chassis_speed.vw=0.006;
}

/**
  * @brief  ��ȡ�����ƶ�ģʽ
  * @param  void
  * @retval TRUE:��еģʽ    false:������ģʽ
  * @attention  
  */
bool CHASSIS_IfActiveMode(void)
{
	if (modeChassis == CHASSIS_MECH_MODE)
	{
		return TRUE;//��е
	}
	else
	{
		return FALSE;//������
	}
}

/**
  * @brief  �����Ƿ�������ģʽ
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
  * @brief  �����Ƿ���Ťƨ��ģʽ
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
  * @brief  �����Ƿ���45��ģʽ
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







