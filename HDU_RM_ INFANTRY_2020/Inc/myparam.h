#ifndef myparam_h
#define myparam_h
#include "main.h"

/********************************������Ϣ****************************************/
		/**************Ӳ���ߴ�*************************/
				#define CHASSIS_DECELE_RATIO  19		      //���ٱ�
				#define LENGTH_A              185         //mm
				#define LENGTH_B              185         //mm
				#define WHEEL_PERIMETER       152         //mm
				
	  /**********ģʽ��������Ƶ�����С***************/
				//��ͨ�Ǹ�����̨��������
				#define NOMOAL_CHASSIS_MAX1 20000
				#define NOMOAL_CHASSIS_MAX2 20000
				#define NOMOAL_CHASSIS_MAX3 20000
				#define NOMOAL_CHASSIS_MAX4 20000
				//���·Ǹ�����̨��������
				#define CLIMBING_CHASSIS_MAX1 20000
				#define CLIMBING_CHASSIS_MAX2 20000
				#define CLIMBING_CHASSIS_MAX3 20000
				#define CLIMBING_CHASSIS_MAX4 20000
				//��ͨ������̨��������
				#define NOMAL_FOLLOW_CHASSIS_MAX1 20000
				#define NOMAL_FOLLOW_CHASSIS_MAX2 20000
				#define NOMAL_FOLLOW_CHASSIS_MAX3 20000
				#define NOMAL_FOLLOW_CHASSIS_MAX4 20000
				//���¸�����̨��������
				#define CLIMBING_FOLLOW_CHASSIS_MAX1 20000
				#define CLIMBING_FOLLOW_CHASSIS_MAX2 20000
				#define CLIMBING_FOLLOW_CHASSIS_MAX3 20000
				#define CLIMBING_FOLLOW_CHASSIS_MAX4 20000
				//��ͨС����/Ťƨ������
				#define NOMAL_GYRO_CHASSIS_MAX1 20000
				#define NOMAL_GYRO_CHASSIS_MAX2 20000
				#define NOMAL_GYRO_CHASSIS_MAX3 20000
				#define NOMAL_GYRO_CHASSIS_MAX4 20000 
				//����С����/Ťƨ������
				#define CLIMBING_GYRO_CHASSIS_MAX1 20000
				#define CLIMBING_GYRO_CHASSIS_MAX2 20000
				#define CLIMBING_GYRO_CHASSIS_MAX3 20000
				#define CLIMBING_GYRO_CHASSIS_MAX4 20000 
				//�ֶ�����ģʽ��������(ֻ������)
				#define CLIMBING_CHASSIS_SZUPUP_MAX1 20000
				#define CLIMBING_CHASSIS_SZUPUP_MAX2 20000
				#define CLIMBING_CHASSIS_SZUPUP_MAX3 20000
				#define CLIMBING_CHASSIS_SZUPUP_MAX4 20000
				//�Զ�����ģʽ��ͨ����
				#define NOMAL_CHASSIS_MISS_MAX1  20000
				#define NOMAL_CHASSIS_MISS_MAX2  20000
				#define NOMAL_CHASSIS_MISS_MAX3  20000
				#define NOMAL_CHASSIS_MISS_MAX4  20000
				//�Զ�����ģʽ��������
				#define CLIMBING_CHASSIS_MISS_MAX1  20000
				#define CLIMBING_CHASSIS_MISS_MAX2  20000
				#define CLIMBING_CHASSIS_MISS_MAX3  20000
				#define CLIMBING_CHASSIS_MISS_MAX4  20000
				//С����ģʽ��ͨ����
				#define NOMAL_CHASSIS_GYROSCOPE_MAX1  20000
				#define NOMAL_CHASSIS_GYROSCOPE_MAX2  20000
				#define NOMAL_CHASSIS_GYROSCOPE_MAX3  20000
				#define NOMAL_CHASSIS_GYROSCOPE_MAX4  20000
				//С����ģʽ��������
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX1  20000
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX2  20000
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX3  20000
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX4  20000
				//���̸�����̨��ͨ����
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX1  20000
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX2  20000
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX3  20000
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX4  20000
				//���̸�����̨��������
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX1  20000
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX2  20000
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX3  20000
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX4  20000
				
				

/**********************************��̨��Ϣ****************************************/
			/***********************Pitch�ᡢYAW����̨��������λ****************************/
					#define GIMBAL_PITCH_ENCODER_MAX 7255    //up
					#define GIMBAL_PITCH_ENCODER_MIDDLE 6780
					#define GIMBAL_PITCH_ENCODER_MIN 6182     //down
					#define GIMBAL_YAW_ENCODER_MAX 6170       //right
					#define GIMBAL_YAW_ENCODER_MIDDLE 4093 
					#define GIMBAL_YAW_ENCODER_MIN 2020        //left
					
					
					
			/***********************YAW����̨���������ض�ֵ******************/
					#define GIMBAL_YAW_ENCODER_MIDDLE1 2700		//���̺���̨������ͬ1��ָ��y
					#define GIMBAL_YAW_ENCODER_MIDDLE2 654		//���̺���̨������ͬ2��ָ��-y
					#define GIMBAL_YAW_ENCODER_FORTYFIVE1 5774	//���̺���̨����45��1��ָ��45��
					#define GIMBAL_YAW_ENCODER_FORTYFIVE2 7822	//���̺���̨����45��2��ָ��135��
					#define GIMBAL_YAW_ENCODER_FORTYFIVE3 1678	//���̺���̨����45��3��ָ��-135��
					#define GIMBAL_YAW_ENCODER_FORTYFIVE4 3726	//���̺���̨����45��4��ָ��-45��
					

					

/***************************������Ϣ****************************************/
			/******************���̵������****************/
			#define   REVOLVER_PID_POSITION_OUTMAX1       2000
			#define   REVOLVER_PID_POSITION_IMAX1         500
			#define   REVOLVER_PID_SPEED_OUTMAX2    20000  
			#define   REVOLVER_PID_SPEED_IMAX2      10000
			/******************����Ӳ���ߴ�******************/
			#define REVOL_SPEED_RATIO   2160       //�����һ��תһȦ,2160ת��ת��,60*36,����Ƶ�ٳ��Բ��̸����Ϳɵ���Ӧ��Ƶ�µ�ת��
			#define 	REVOL_SPEED_GRID      12			//���̸���
			#define    	AN_BULLET         (24576.0f)		//�����ӵ����λ������ֵ(���ֵ�ò�Ѽ)
			


/********************ң����/���̲���****************************/
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX 300.0f	//���̸�����̨ģʽ������vx  Խ��������ԽС
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY 300.0f	//���̸�����̨ģʽ������vy  Խ��������ԽС


#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX 300.0f	//���̲�������̨ģʽ������vx  Խ��������ԽС
#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY 300.0f	//���̲�������̨ģʽ������vy  Խ��������ԽС


#define SENSITIVITY_REMOTE_GIMBAL_YAW 50.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_REMOTE_GIMBAL_PITCH 50.0f		//��̨������pitch�ᣬԽ��������ԽС

#define SENSITIVITY_REMOTE_GIMBAL_YAW_IMU 1140.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU 1140.0f		//��̨������pitch�ᣬԽ��������ԽС

/***********************�Ӿ�������*************************************/

#define SENSITIVITY_VISION_GIMBAL_YAW_ENCODER 700.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER 700.0f		//��̨������pitch�ᣬԽ��������ԽС

/********************��������****************************/
#define FIRING_RATE -600 //3��ÿ��
#define INITIAL_SPEED_MAX 0 //���������ٶ�

#define hero_fire_l -15000;
#define hero_fire_r  15000;
#define hero_fire_rpm 10;


#endif


