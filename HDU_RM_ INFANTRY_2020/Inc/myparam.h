#ifndef myparam_h
#define myparam_h
#include "main.h"

/***********************底盘信息****************************************/
#define CHASSIS_DECELE_RATIO  19		//减速比
#define LENGTH_A 185         //mm
#define LENGTH_B 185         //mm
#define WHEEL_PERIMETER 152  //mm
/***********************Pitch轴云台编码器限位****************************/
#define GIMBAL_PITCH_ENCODER_MAX 7255
#define GIMBAL_PITCH_ENCODER_MIN 6182
#define GIMBAL_PITCH_ENCODER_MIDDLE 6780
/***********************YAW轴云台编码器的特定值******************/
#define GIMBAL_YAW_ENCODER_MIDDLE1 2700		//底盘和云台朝向相同1，指向＋y
#define GIMBAL_YAW_ENCODER_MIDDLE2 654		//底盘和云台朝向相同2，指向-y
#define GIMBAL_YAW_ENCODER_FORTYFIVE1 5774	//底盘和云台朝向45°1，指向45°
#define GIMBAL_YAW_ENCODER_FORTYFIVE2 7822	//底盘和云台朝向45°2，指向135°
#define GIMBAL_YAW_ENCODER_FORTYFIVE3 1678	//底盘和云台朝向45°3，指向-135°
#define GIMBAL_YAW_ENCODER_FORTYFIVE4 3726	//底盘和云台朝向45°4，指向-45°
/**********各个模式下各个电机的限制电流大小***************/
//普通非跟随云台底盘限流
#define NOMOAL_CHASSIS_MAX1 20000
#define NOMOAL_CHASSIS_MAX2 20000
#define NOMOAL_CHASSIS_MAX3 20000
#define NOMOAL_CHASSIS_MAX4 20000
//爬坡非跟随云台底盘限流
#define CLIMBING_CHASSIS_MAX1 20000
#define CLIMBING_CHASSIS_MAX2 20000
#define CLIMBING_CHASSIS_MAX3 20000
#define CLIMBING_CHASSIS_MAX4 20000
//普通跟随云台底盘限流
#define NOMAL_FOLLOW_CHASSIS_MAX1 20000
#define NOMAL_FOLLOW_CHASSIS_MAX2 20000
#define NOMAL_FOLLOW_CHASSIS_MAX3 20000
#define NOMAL_FOLLOW_CHASSIS_MAX4 20000
//爬坡跟随云台底盘限流
#define CLIMBING_FOLLOW_CHASSIS_MAX1 20000
#define CLIMBING_FOLLOW_CHASSIS_MAX2 20000
#define CLIMBING_FOLLOW_CHASSIS_MAX3 20000
#define CLIMBING_FOLLOW_CHASSIS_MAX4 20000
//普通小陀螺/扭屁股限流
#define NOMAL_GYRO_CHASSIS_MAX1 20000
#define NOMAL_GYRO_CHASSIS_MAX2 20000
#define NOMAL_GYRO_CHASSIS_MAX3 20000
#define NOMAL_GYRO_CHASSIS_MAX4 20000 
//爬坡小陀螺/扭屁股限流
#define CLIMBING_GYRO_CHASSIS_MAX1 20000
#define CLIMBING_GYRO_CHASSIS_MAX2 20000
#define CLIMBING_GYRO_CHASSIS_MAX3 20000
#define CLIMBING_GYRO_CHASSIS_MAX4 20000 

/********************遥控器/键盘参数****************************/
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX 300.0f	//底盘跟随云台模式灵敏度vx  越大灵敏度越小
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY 300.0f	//底盘跟随云台模式灵敏度vy  越大灵敏度越小


#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX 300.0f	//底盘不跟随云台模式灵敏度vx  越大灵敏度越小
#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY 300.0f	//底盘不跟随云台模式灵敏度vy  越大灵敏度越小


#define SENSITIVITY_REMOTE_GIMBAL_YAW 50.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_REMOTE_GIMBAL_PITCH 50.0f		//云台灵敏度pitch轴，越大灵敏度越小

#define SENSITIVITY_REMOTE_GIMBAL_YAW_IMU 1140.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU 1140.0f		//云台灵敏度pitch轴，越大灵敏度越小

/***********************视觉灵敏度*************************************/

#define SENSITIVITY_VISION_GIMBAL_YAW_ENCODER 700.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER 700.0f		//云台灵敏度pitch轴，越大灵敏度越小

/********************发弹参数****************************/
#define FIRING_RATE -600 //3发每秒
#define INITIAL_SPEED_MAX 0 //最大射击初速度

#define hero_fire_l -15000;
#define hero_fire_r  15000;
#define hero_fire_rpm 10;


#endif


