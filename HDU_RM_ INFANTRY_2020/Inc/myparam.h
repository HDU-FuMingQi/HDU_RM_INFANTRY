#ifndef myparam_h
#define myparam_h
#include "main.h"

/***********************������Ϣ****************************************/
#define CHASSIS_DECELE_RATIO  19		//���ٱ�
#define LENGTH_A 185         //mm
#define LENGTH_B 185         //mm
#define WHEEL_PERIMETER 152  //mm
/***********************Pitch����̨��������λ****************************/
#define GIMBAL_PITCH_ENCODER_MAX 7255
#define GIMBAL_PITCH_ENCODER_MIN 6182
#define GIMBAL_PITCH_ENCODER_MIDDLE 6780
/***********************YAW����̨���������ض�ֵ******************/
#define GIMBAL_YAW_ENCODER_MIDDLE1 2700		//���̺���̨������ͬ1��ָ��y
#define GIMBAL_YAW_ENCODER_MIDDLE2 654		//���̺���̨������ͬ2��ָ��-y
#define GIMBAL_YAW_ENCODER_FORTYFIVE1 5774	//���̺���̨����45��1��ָ��45��
#define GIMBAL_YAW_ENCODER_FORTYFIVE2 7822	//���̺���̨����45��2��ָ��135��
#define GIMBAL_YAW_ENCODER_FORTYFIVE3 1678	//���̺���̨����45��3��ָ��-135��
#define GIMBAL_YAW_ENCODER_FORTYFIVE4 3726	//���̺���̨����45��4��ָ��-45��
/**********����ģʽ�¸�����������Ƶ�����С***************/
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


