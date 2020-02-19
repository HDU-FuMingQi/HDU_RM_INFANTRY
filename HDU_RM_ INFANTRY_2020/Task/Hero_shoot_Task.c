//#include "Hero_shoot_Task.h"

//int8_t fire_mode = hero_stop;

//const static fp32 hero_Fire_Motor_Position_pid[3] = {0, 0, 0};
//const static fp32 hero_Fire_Motor_Speed_pid[3] = {120, 20, 8};

//const static fp32 hero_friction_Motor_l_Speed_pid[3] = {100, 10, 0};
//const static fp32 hero_friction_Motor_l_Position_pid[3] = {0, 0, 0};

//const static fp32 hero_friction_Motor_r_Speed_pid[3] = {100, 10, 0};
//const static fp32 hero_friction_Motor_r_Position_pid[3] = {0, 0, 0};



//void hero_shoot_fun(void const * argument)
//{

//	portTickType currentTime;
//	//hero shoot motor initial
//	Motor_Init2(&hero_Fire_Motor, 9, hero_Fire_Motor_Position_pid, 2000, 500,hero_Fire_Motor_Speed_pid, 30000, 10000);
//	Motor_Init2(&hero_friction_Motor_l, 1, hero_friction_Motor_l_Position_pid, 2000, 500, hero_friction_Motor_l_Speed_pid, 20000, 10000);
//	Motor_Init2(&hero_friction_Motor_r, 2, hero_friction_Motor_r_Position_pid, 2000, 500, hero_friction_Motor_r_Speed_pid, 20000, 10000);

//  for(;;)
//  {
//    currentTime = xTaskGetTickCount();

//   	hero_Set_Fire_Mode();
//		hero_Moto_Speed_Set();

//		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS); //¾ø¶ÔÑÓÊ±
//  }
//  /* USER CODE END hero_shoot_fun */
//}









//void hero_Set_Fire_Mode(void)
//{
//	switch (rc.sw2)
//	{
//	case 1:
//		fire_mode = hero_single;
//		break;
//	case 2:
//		fire_mode = hero_continuity;
//		break;
//	default:
//		fire_mode = hero_stop;
//		break;
//	}
//}

//void hero_Moto_Speed_Set()
//{
//	switch (fire_mode)
//	{
//		case hero_stop :
//			hero_shoot_stop();
//		  break;
//		case hero_single :
//			hero_shoot_stop();
//			break;
//		case hero_continuity:
//			hero_shoot_continuity();
//		  break;
//		default:
//			hero_shoot_stop();
//			break;
//	}
//	
//}

//void hero_shoot_stop()
//{
//	hero_Fire_Motor.motor_value->target_speed_rpm = 0;
//	hero_friction_Motor_l.motor_value->target_speed_rpm = 0;
//	hero_friction_Motor_r.motor_value->target_speed_rpm = 0;
//	
//	PID_Calc(&hero_Fire_Motor.Motor_PID_Speed, hero_Fire_Motor.motor_value->speed_rpm,
//				 hero_Fire_Motor.motor_value->target_speed_rpm);
//	PID_Calc(&hero_friction_Motor_l.Motor_PID_Speed, hero_friction_Motor_l.motor_value->speed_rpm,
//				 hero_friction_Motor_l.motor_value->target_speed_rpm);
//	PID_Calc(&hero_friction_Motor_r.Motor_PID_Speed, hero_friction_Motor_r.motor_value->speed_rpm,
//				 hero_friction_Motor_r.motor_value->target_speed_rpm);
//	
//	set_moto91011_current(&hcan2, hero_Fire_Motor.Motor_PID_Speed.out, 0, 0);
//	
//	if(hero_friction_Motor_l.motor_value->speed_rpm<100)
//		 set_moto1234_current(&hcan2, 0, 0, 0, 0);
//	else
//		set_moto1234_current(&hcan2, hero_friction_Motor_l.Motor_PID_Speed.out, hero_friction_Motor_r.Motor_PID_Speed.out, 0, 0);
//}

//void hero_shoot_continuity()
//{
//	hero_Fire_Motor.motor_value->target_speed_rpm = hero_fire_rpm;
//	hero_friction_Motor_l.motor_value->target_speed_rpm = hero_fire_l;
//	hero_friction_Motor_r.motor_value->target_speed_rpm = hero_fire_r ;
//	
//	PID_Calc(&hero_Fire_Motor.Motor_PID_Speed, hero_Fire_Motor.motor_value->speed_rpm,
//				 hero_Fire_Motor.motor_value->target_speed_rpm);
//	PID_Calc(&hero_friction_Motor_l.Motor_PID_Speed, hero_friction_Motor_l.motor_value->speed_rpm,
//				 hero_friction_Motor_l.motor_value->target_speed_rpm);
//	PID_Calc(&hero_friction_Motor_r.Motor_PID_Speed, hero_friction_Motor_r.motor_value->speed_rpm,
//				 hero_friction_Motor_r.motor_value->target_speed_rpm);
//	
//	set_moto91011_current(&hcan2, hero_Fire_Motor.Motor_PID_Speed.out, 0, 0);
//	set_moto1234_current(&hcan2, hero_friction_Motor_l.Motor_PID_Speed.out, hero_friction_Motor_r.Motor_PID_Speed.out, 0, 0);
//	 
//}