#ifndef myparam_h
#define myparam_h
#include "main.h"

/********************************底盘信息****************************************/
		/**************硬件尺寸*************************/
				#define CHASSIS_DECELE_RATIO  19		      //减速比
				#define LENGTH_A              185         //mm
				#define LENGTH_B              185         //mm
				#define WHEEL_PERIMETER       152         //mm
				
	  /**********模式电机的限制电流大小***************/
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
				//手动爬坡模式地盘限流(只有爬坡)
				#define CLIMBING_CHASSIS_SZUPUP_MAX1 20000
				#define CLIMBING_CHASSIS_SZUPUP_MAX2 20000
				#define CLIMBING_CHASSIS_SZUPUP_MAX3 20000
				#define CLIMBING_CHASSIS_SZUPUP_MAX4 20000
				//自动闪避模式普通限流
				#define NOMAL_CHASSIS_MISS_MAX1  20000
				#define NOMAL_CHASSIS_MISS_MAX2  20000
				#define NOMAL_CHASSIS_MISS_MAX3  20000
				#define NOMAL_CHASSIS_MISS_MAX4  20000
				//自动闪避模式爬坡限流
				#define CLIMBING_CHASSIS_MISS_MAX1  20000
				#define CLIMBING_CHASSIS_MISS_MAX2  20000
				#define CLIMBING_CHASSIS_MISS_MAX3  20000
				#define CLIMBING_CHASSIS_MISS_MAX4  20000
				//小陀螺模式普通限流
				#define NOMAL_CHASSIS_GYROSCOPE_MAX1  20000
				#define NOMAL_CHASSIS_GYROSCOPE_MAX2  20000
				#define NOMAL_CHASSIS_GYROSCOPE_MAX3  20000
				#define NOMAL_CHASSIS_GYROSCOPE_MAX4  20000
				//小陀螺模式爬坡限流
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX1  20000
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX2  20000
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX3  20000
				#define CLIMBING_CHASSIS_GYROSCOPE_MAX4  20000
				//底盘跟随云台普通限流
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX1  20000
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX2  20000
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX3  20000
				#define NOMAL_CHASSIS_FOLLOW_GIMBAL_MAX4  20000
				//底盘跟随云台爬坡限流
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX1  20000
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX2  20000
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX3  20000
				#define CLIMBING_CHASSIS_FOLLOW_GIMBAL_MAX4  20000
				
				

/**********************************云台信息****************************************/
			/***********************Pitch轴、YAW轴云台编码器限位****************************/
					#define GIMBAL_PITCH_ENCODER_MAX 7255    //up
					#define GIMBAL_PITCH_ENCODER_MIDDLE 6780
					#define GIMBAL_PITCH_ENCODER_MIN 6182     //down
					#define GIMBAL_YAW_ENCODER_MAX 6170       //right
					#define GIMBAL_YAW_ENCODER_MIDDLE 4093 
					#define GIMBAL_YAW_ENCODER_MIN 2020        //left
					
					
					
			/***********************YAW轴云台编码器的特定值******************/
					#define GIMBAL_YAW_ENCODER_MIDDLE1 2700		//底盘和云台朝向相同1，指向＋y
					#define GIMBAL_YAW_ENCODER_MIDDLE2 654		//底盘和云台朝向相同2，指向-y
					#define GIMBAL_YAW_ENCODER_FORTYFIVE1 5774	//底盘和云台朝向45°1，指向45°
					#define GIMBAL_YAW_ENCODER_FORTYFIVE2 7822	//底盘和云台朝向45°2，指向135°
					#define GIMBAL_YAW_ENCODER_FORTYFIVE3 1678	//底盘和云台朝向45°3，指向-135°
					#define GIMBAL_YAW_ENCODER_FORTYFIVE4 3726	//底盘和云台朝向45°4，指向-45°
					

					

/***************************拨盘信息****************************************/
			/******************拨盘电机限流****************/
			#define   REVOLVER_PID_POSITION_OUTMAX1       2000
			#define   REVOLVER_PID_POSITION_IMAX1         500
			#define   REVOLVER_PID_SPEED_OUTMAX2    20000  
			#define   REVOLVER_PID_SPEED_IMAX2      10000
			/******************拨盘硬件尺寸******************/
			#define REVOL_SPEED_RATIO   2160       //电机轴一秒转一圈,2160转子转速,60*36,乘射频再除以拨盘格数就可得相应射频下的转速
			#define 	REVOL_SPEED_GRID      12			//拨盘格数
			#define    	AN_BULLET         (24576.0f)		//单个子弹电机位置增加值(这个值得测鸭)
			


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


