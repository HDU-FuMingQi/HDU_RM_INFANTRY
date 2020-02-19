#include "OLED_Task.h"
#include "mytype.h"
#include "bsp_motor.h"


//#include "oledfont.h"
//该任务用于显示OLED，读取oled上的按键信息
void OLED_Change_Param(void);
void Oled_ShowIMU(void);
void Oled_Show_Manu(void);
void OLED_Show_Info(void);
void OLED_Change_Param(void);
void Oled_ShowMotorSet(void);
void Oled_ShowJY901(void);
void Oled_ShowRC(void);
void Oled_ShowRC_PC(void);
void Oled_ShowMotor(void);
void oled_LOGO(void);
void Oled_ShowGM6020Set(void);
void Oled_ShowGM3508Set_l(void);
void Oled_ShowGM3508Set_r(void);
void ShowOLEDFun(void const * argument)

{
	/* USER CODE BEGIN OLED_Fun */
	  /* Infinite loop */
	//初始化OLED
	
	  oled_LOGO();
	  //oled清屏
	  oled_clear(Pen_Clear);

	  for(;;)
	  {
		Oled_Show_Manu();
	    osDelay(20);
	  }
	  /* USER CODE END OLED_Fun */
}
void Oled_Show_Manu()
{
	static uint8_t col_num=1;
	uint8_t key;
	key=KEY_Scan();
	for(uint8_t loop=0;loop!=col_num-1;loop++)
	{
		oled_showstring(loop,1,"  ");
	}
	for(uint8_t loop=col_num;loop!=5;loop++)
	{
		oled_showstring(loop,1,"  ");
	}
	oled_showstring(col_num-1,1,"->");
	oled_showstring(0,5,"Information");
	oled_showstring(1,5,"Param");
	oled_showfloat(2,5,hero_Fire_Motor.Motor_PID_Speed.Kp,3,6);
	oled_showfloat(3,5,hero_Fire_Motor.Motor_PID_Speed.Ki,3,6);
//	oled_showfloat(2,5,VisionValue.vision_pitch_value.value,3,6);
//	oled_showfloat(3,5,VisionValue.vision_yaw_value.value,3,6);
	// oled_shownum(2,5,ADCxConvertedValue,0,6);

	switch(key)
	{
	case key_up_pres:col_num--;if(col_num<=1)col_num=1;
		break;
	case key_down_pres:col_num++;if(col_num>=2)col_num=2;
		break;
	case key_middle_pres:
		oled_clear(Pen_Clear);
		switch(col_num)
		{
		case 1:
			OLED_Show_Info();
			break;
		case 2:
			OLED_Change_Param();
			break;
		}
		break;
	}
}
void OLED_Show_Info(void)
{
	static uint8_t oled_num=1;
	for(;;)
	{
		switch(KEY_Scan())
		{
		case key_left_pres:oled_num--;oled_clear(Pen_Clear);break;
		case key_right_pres:oled_num++;oled_clear(Pen_Clear);break;
		case key_middle_pres:oled_clear(Pen_Clear);return;
		default:break;
		}
		if(oled_num<=1)
			oled_num=1;
		if(oled_num>=shownothing)
			oled_num=shownothing-1;


		switch(oled_num)
		{
		case showRc:Oled_ShowRC();break;
		case showPC:Oled_ShowRC_PC();break;
		case showMotor:Oled_ShowMotor();break;
		case showIMU:Oled_ShowIMU();break;
		case showMotorSet:Oled_ShowMotorSet();break;
		case showGM6020Set:Oled_ShowGM6020Set();break;
		case showJY901:Oled_ShowJY901();break;

		}
		osDelay(1);
	}
}

#define _PID1 Chassis_Motor1.Motor_PID_Speed
#define _PID2 Chassis_Motor2.Motor_PID_Speed
#define _PID3 Chassis_Motor3.Motor_PID_Speed
#define _PID4 Chassis_Motor4.Motor_PID_Speed
#define _PID5 Gimbal_MotorYaw.Motor_PID_Speed
#define _PID6 Chassis_Motor4.Motor_PID_Speed
#define _PID7 Chassis_Motor4.Motor_PID_Speed
#define _PID8 Chassis_Motor4.Motor_PID_Speed
#define _PID9 Chassis_Motor4.Motor_PID_Speed
#define _PI10 Chassis_Motor4.Motor_PID_Speed
#define _PI11 Chassis_Motor4.Motor_PID_Speed
#define _PI12 Chassis_Motor4.Motor_PID_Speed
#define _PI13 Chassis_Motor4.Motor_PID_Speed
#define _PI14 Chassis_Motor4.Motor_PID_Speed

#define _PI15 hero_Fire_Motor.Motor_PID_Speed
#define _PI16 hero_friction_Motor_l.Motor_PID_Speed
#define _PI17 hero_friction_Motor_r.Motor_PID_Speed
void OLED_Change_Param(void)
{
	static uint8_t col_num=1;		//p还是i还是d
	static uint8_t mode=1;
	static uint8_t param_num=1;		//哪一个
	static uint8_t param_lock=0;
	for(;;)
	{

		for(uint8_t loop=0;loop!=(param_num-1)%5;loop++)
		{
			oled_showstring(loop,0,"  ");
		}
		if(param_num%5)
		{
			for(uint8_t loop=(param_num)%5;loop!=5;loop++)
			{
				oled_showstring(loop,0,"  ");
			}
		}
		oled_showstring((param_num-1)%5,0,"->");

		if(mode==2)
		{
			for(uint8_t loop=0;loop!=(col_num-1)%5;loop++)
			{
				oled_showstring(loop,9,"  ");
			}
			for(uint8_t loop=(col_num)%5;loop!=5;loop++)
			{
				oled_showstring(loop,9,"  ");
			}
			if(!param_lock)
				oled_showstring((col_num-1)%5,9,"->");
			else
				oled_showstring((col_num-1)%5,9,"**");
		}

		if(param_num<=5)
		{
			oled_showstring(0,3,"PID1");
			oled_showstring(1,3,"PID2");
			oled_showstring(2,3,"PID3");
			oled_showstring(3,3,"PID4");
			oled_showstring(4,3,"PID5");
		}
		else if(param_num<=10)
		{
			oled_showstring(0,3,"PID6");
			oled_showstring(1,3,"PID7");
			oled_showstring(2,3,"PID8");
			oled_showstring(3,3,"PID9");
			oled_showstring(4,3,"PI10");
		}
		else if(param_num<=15)
		{
			oled_showstring(0,3,"PI11");
			oled_showstring(1,3,"PI12");
			oled_showstring(2,3,"PI13");
			oled_showstring(3,3,"PI14");
			oled_showstring(4,3,"PI15");
		}
		oled_showstring(0,12,"P");
		oled_showstring(1,12,"I");
		oled_showstring(2,12,"D");
		oled_showstring(3,12,"O");
		oled_showstring(4,12,"S");
//		oled_shownum(4,12,param_num,1,2);
//		oled_shownum(4,16,col_num,1,2);
		switch(param_num)
		{
		case 1:
			oled_showfloat(0,14,_PID1.Kp,3,2);
			oled_showfloat(1,14,_PID1.Ki,3,2);
			oled_showfloat(2,14,_PID1.Kd,3,2);
			oled_showfloat(3,14,_PID1.out,5,0);
			oled_showfloat(4,14,_PID1.set,5,0);
			break;
		case 2:
			oled_showfloat(0,14,_PID2.Kp,3,2);
			oled_showfloat(1,14,_PID2.Ki,3,2);
			oled_showfloat(2,14,_PID2.Kd,3,2);
			oled_showfloat(3,14,_PID2.out,3,2);
			oled_showfloat(4,14,_PID2.set,5,0);
			break;
		case 3:
			oled_showfloat(0,14,_PID3.Kp,3,2);
			oled_showfloat(1,14,_PID3.Ki,3,2);
			oled_showfloat(2,14,_PID3.Kd,3,2);
			oled_showfloat(3,14,_PID3.out,3,2);
			oled_showfloat(4,14,_PID3.set,5,0);
			break;
		case 4:
			oled_showfloat(0,14,_PID4.Kp,3,2);
			oled_showfloat(1,14,_PID4.Ki,3,2);
			oled_showfloat(2,14,_PID4.Kd,3,2);
			oled_showfloat(3,14,_PID4.out,5,0);
			oled_showfloat(4,14,_PID4.set,5,0);
			break;
		case 5:
			oled_showfloat(0,14,_PID5.Kp,3,2);
			oled_showfloat(1,14,_PID5.Ki,3,2);
			oled_showfloat(2,14,_PID5.Kd,3,2);
			oled_showfloat(3,14,_PID5.out,3,2);
			oled_showfloat(4,14,_PID5.set,5,0);
			break;
		case 6:
			oled_showfloat(0,14,_PID6.Kp,3,2);
			oled_showfloat(1,14,_PID6.Ki,3,2);
			oled_showfloat(2,14,_PID6.Kd,3,2);
			oled_showfloat(3,14,_PID6.out,3,2);
			oled_showfloat(4,14,_PID6.set,5,0);
			break;
		case 7:
			oled_showfloat(0,14,_PID7.Kp,3,2);
			oled_showfloat(1,14,_PID7.Ki,3,2);
			oled_showfloat(2,14,_PID7.Kd,3,2);
			oled_showfloat(3,14,_PID7.out,3,2);
			oled_showfloat(4,14,_PID7.set,5,0);
			break;
		case 8:
			oled_showfloat(0,14,_PID8.Kp,3,2);
			oled_showfloat(1,14,_PID8.Ki,3,2);
			oled_showfloat(2,14,_PID8.Kd,3,2);
			oled_showfloat(3,14,_PID8.out,3,2);
			oled_showfloat(4,14,_PID8.set,5,0);
			break;
		case 9:
			oled_showfloat(0,14,_PID9.Kp,3,2);
			oled_showfloat(1,14,_PID9.Ki,3,2);
			oled_showfloat(2,14,_PID9.Kd,3,2);
			oled_showfloat(3,14,_PID9.out,3,2);
			oled_showfloat(4,14,_PID9.set,5,0);
			break;
		case 10:
			oled_showfloat(0,14,_PI10.Kp,3,2);
			oled_showfloat(1,14,_PI10.Ki,3,2);
			oled_showfloat(2,14,_PI10.Kd,3,2);
			oled_showfloat(3,14,_PI10.out,3,2);
			oled_showfloat(4,14,_PI10.set,5,0);
			break;
		case 11:
			oled_showfloat(0,14,_PI11.Kp,3,2);
			oled_showfloat(1,14,_PI11.Ki,3,2);
			oled_showfloat(2,14,_PI11.Kd,3,2);
			oled_showfloat(3,14,_PI11.out,3,2);
			oled_showfloat(4,14,_PI11.set,5,0);
			break;
		case 12:
			oled_showfloat(0,14,_PI12.Kp,3,2);
			oled_showfloat(1,14,_PI12.Ki,3,2);
			oled_showfloat(2,14,_PI12.Kd,3,2);
			oled_showfloat(3,14,_PI12.out,5,0);
			oled_showfloat(4,14,_PI12.set,5,0);
			break;
		case 13:
			oled_showfloat(0,14,_PI13.Kp,3,2);
			oled_showfloat(1,14,_PI13.Ki,3,2);
			oled_showfloat(2,14,_PI13.Kd,3,2);
			oled_showfloat(3,14,_PI13.out,5,0);
			oled_showfloat(4,14,_PI13.set,5,0);
			break;
		case 14:
			oled_showfloat(0,14,_PI14.Kp,3,2);
			oled_showfloat(1,14,_PI14.Ki,3,2);
			oled_showfloat(2,14,_PI14.Kd,3,2);
			oled_showfloat(3,14,_PI14.out,5,0);
			oled_showfloat(4,14,_PI14.set,5,0);
			break;
		case 15:
			oled_showfloat(0,14,_PI15.Kp,3,2);
			oled_showfloat(1,14,_PI15.Ki,3,2);
			oled_showfloat(2,14,_PI15.Kd,3,2);
			oled_showfloat(3,14,_PI15.out,5,0);
			oled_showfloat(4,14,_PI15.set,5,0);
			break;
		}
		if(mode==2)
		{
			switch(KEY_Scan())
			{
			case key_middle_pres:param_lock=!param_lock;break;
			case key_left_pres:mode=1;oled_clear(Pen_Clear);break;
			case key_up_pres:
				if(!param_lock)
				{
					col_num--;
					if(col_num<=1)
						col_num=1;
				}
				else
				{
					switch(col_num)
					{
					case 1:
						if(param_num==1)
							_PID1.Kp+=0.1f;
						else if(param_num==2)
							_PID2.Kp+=0.1f;
						else if(param_num==3)
							_PID3.Kp+=0.1f;
						else if(param_num==4)
							_PID4.Kp+=0.1f;
						else if(param_num==5)
							_PID5.Kp+=0.1f;
						else if(param_num==6)
							_PID6.Kp+=0.1f;
						else if(param_num==7)
							_PID7.Kp+=0.1f;
						else if(param_num==8)
							_PID8.Kp+=0.1f;
						else if(param_num==9)
							_PID9.Kp+=0.1f;
						else if(param_num==10)
							_PI10.Kp+=0.1f;
						else if(param_num==11)
							_PI11.Kp+=0.1f;
						else if(param_num==12)
							_PI12.Kp+=0.1f;
						else if(param_num==13)
							_PI13.Kp+=1;
						else if(param_num==14)
							_PI14.Kp+=1;
						break;

					case 2:
						if(param_num==1)
							_PID1.Ki+=0.1f;
						else if(param_num==2)
							_PID2.Ki+=0.1f;
						else if(param_num==3)
							_PID3.Ki+=0.1f;
						else if(param_num==4)
							_PID4.Ki+=0.1f;
						else if(param_num==5)
							_PID5.Ki+=0.1f;
						else if(param_num==6)
							_PID6.Ki+=0.1f;
						else if(param_num==7)
							_PID7.Ki+=0.1f;
						else if(param_num==8)
							_PID8.Ki+=0.1f;
						else if(param_num==9)
							_PID9.Ki+=0.1f;
						else if(param_num==10)
							_PI10.Kp+=0.1f;
						else if(param_num==11)
							_PI11.Ki+=0.1f;
						else if(param_num==12)
							_PI12.Ki+=0.1f;
						else if(param_num==13)
							_PI13.Ki+=0.1f;
						else if(param_num==14)
							_PI14.Ki+=0.1f;
						break;

					case 3:
						if(param_num==1)
							_PID1.Kd+=0.01f;
						else if(param_num==2)
							_PID2.Kd+=0.01f;
						else if(param_num==3)
							_PID3.Kd+=0.01f;
						else if(param_num==4)
							_PID4.Kd+=0.01f;
						else if(param_num==5)
							_PID5.Kd+=0.01f;
						else if(param_num==6)
							_PID6.Kd+=0.01f;
						else if(param_num==7)
							_PID7.Kd+=0.01f;
						else if(param_num==8)
							_PID8.Kd+=0.01f;
						else if(param_num==9)
							_PID9.Kd+=0.01f;
						else if(param_num==10)
							_PI10.Kd+=0.01f;
						else if(param_num==11)
							_PI11.Kd+=0.01f;
						else if(param_num==12)
							_PI12.Kd+=0.1f;
						else if(param_num==13)
							_PI13.Kd+=0.1f;
						else if(param_num==14)
							_PI14.Kd+=0.1f;
						break;
					}
				}
				break;
			case key_down_pres:
				if(!param_lock)
				{
					col_num++;
					if(col_num>=3)
						col_num=3;
				}
				else
				{
					switch(col_num)
					{
					case 1:
						if(param_num==1)
							_PID1.Kp-=0.1f;
						else if(param_num==2)
							_PID2.Kp-=0.1f;
						else if(param_num==3)
							_PID3.Kp-=0.1f;
						else if(param_num==4)
							_PID4.Kp-=0.1f;
						else if(param_num==5)
							_PID5.Kp-=0.1f;
						else if(param_num==6)
							_PID6.Kp-=0.1f;
						else if(param_num==7)
							_PID7.Kp-=0.1f;
						else if(param_num==8)
							_PID8.Kp-=0.1f;
						else if(param_num==9)
							_PID9.Kp-=0.1f;
						else if(param_num==10)
							_PI10.Kp-=0.1f;
						else if(param_num==11)
							_PI11.Kp-=0.1f;
						else if(param_num==12)
							_PI12.Kp-=0.1f;
						else if(param_num==13)
							_PI13.Kp-=1;
						else if(param_num==14)
							_PI14.Kp-=1;
						break;

					case 2:
						if(param_num==1)
							_PID1.Ki-=0.1f;
						else if(param_num==2)
							_PID2.Ki-=0.1f;
						else if(param_num==3)
							_PID3.Ki-=0.1f;
						else if(param_num==4)
							_PID4.Ki-=0.1f;
						else if(param_num==5)
							_PID5.Ki-=0.1f;
						else if(param_num==6)
							_PID6.Ki-=0.1f;
						else if(param_num==7)
							_PID7.Ki-=0.1f;
						else if(param_num==8)
							_PID8.Ki-=0.1f;
						else if(param_num==9)
							_PID9.Ki-=0.1f;
						else if(param_num==10)
							_PI10.Kp-=0.1f;
						else if(param_num==11)
							_PI11.Ki-=0.1f;
						else if(param_num==12)
							_PI12.Ki-=0.1f;
						else if(param_num==13)
							_PI13.Ki-=0.1f;
						else if(param_num==14)
							_PI14.Ki-=0.1f;
						break;

					case 3:
						if(param_num==1)
							_PID1.Kd-=0.01f;
						else if(param_num==2)
							_PID2.Kd-=0.01f;
						else if(param_num==3)
							_PID3.Kd-=0.01f;
						else if(param_num==4)
							_PID4.Kd-=0.01f;
						else if(param_num==5)
							_PID5.Kd-=0.01f;
						else if(param_num==6)
							_PID6.Kd-=0.01f;
						else if(param_num==7)
							_PID7.Kd-=0.01f;
						else if(param_num==8)
							_PID8.Kd-=0.01f;
						else if(param_num==9)
							_PID9.Kd-=0.01f;
						else if(param_num==10)
							_PI10.Kd-=0.01f;
						else if(param_num==11)
							_PI11.Kd-=0.01f;
						else if(param_num==12)
							_PI12.Kd-=0.1f;
						else if(param_num==13)
							_PI13.Kd-=0.1f;
						else if(param_num==14)
							_PI14.Kd-=0.1f;
						break;
					}
				}
				break;
			default:break;
			}
		}
		else if(mode==1)
		{
			switch(KEY_Scan())
			{
			case key_middle_pres:oled_clear(Pen_Clear);return;
			case key_right_pres:mode=2;oled_clear(Pen_Clear);break;
			case key_up_pres:param_num--;if(param_num<=1)param_num=1;oled_clear(Pen_Clear);break;
			case key_down_pres:param_num++;oled_clear(Pen_Clear);break;
			default:break;
			}
		}
		osDelay(1);
	}
}
void Oled_ShowIMU(void)
{
	oled_showstring(0,0,"wx");
	oled_showfloat(0,2,IMU_Vale.GyroX,3,1);
	oled_showstring(0,8,"wy");
	oled_showfloat(0,10,IMU_Vale.GyroY,3,1);
	oled_showstring(1,0,"wz");
	oled_showfloat(1,2,IMU_Vale.GyroZ,4,1);

	oled_showstring(2,0,"Pi");
	oled_showfloat(2,2,IMU_Vale.Pitch,3,1);
	oled_showstring(2,8,"Ro");
	oled_showfloat(2,10,IMU_Vale.Roll,3,1);
	oled_showstring(3,0,"Ya");
	oled_showfloat(3,2,IMU_Vale.Yaw,4,1);

//	oled_showstring(4,0,"te");
//	oled_shownum(4,2,IMU_Vale.temp,1,4);
}

void Oled_ShowGM6020Set(void)
{
	oled_showstring(0,0,"p");
	oled_showfloat(0,1,hero_Fire_Motor.Motor_PID_Speed.Kp,4,1);
	oled_showstring(0,8,"i");
	oled_showfloat(0,10,hero_Fire_Motor.Motor_PID_Speed.Ki,3,1);
	oled_showstring(1,0,"d");
	oled_showfloat(1,2,hero_Fire_Motor.Motor_PID_Speed.Kd,4,1);

	oled_showstring(2,0,"t");
	oled_showfloat(2,2,hero_Fire_Motor.motor_value->target_speed_rpm,3,1);
	oled_showstring(2,8,"n");
	oled_showfloat(2,10,hero_Fire_Motor.motor_value->speed_rpm,3,1);
	oled_showstring(3,0,"o");
	oled_showfloat(3,2,hero_Fire_Motor.Motor_PID_Speed.out,4,1);
	
	oled_refresh_gram();

//	oled_showstring(4,0,"te");
//	oled_shownum(4,2,IMU_Vale.temp,1,4);
}

void Oled_ShowGM3508Set_l(void)
{
	oled_showstring(0,0,"Kp");
	oled_showfloat(0,2,hero_friction_Motor_l.Motor_PID_Speed.Kp,3,1);
	oled_showstring(0,8,"Ki");
	oled_showfloat(0,10,hero_friction_Motor_l.Motor_PID_Speed.Ki,3,1);
	oled_showstring(1,0,"Kd");
	oled_showfloat(1,2,hero_friction_Motor_l.Motor_PID_Speed.Kd,4,1);

	oled_showstring(2,0,"t");
	oled_showfloat(2,2,hero_friction_Motor_l.motor_value->target_speed_rpm,3,1);
	oled_showstring(2,8,"n");
	oled_showfloat(2,10,hero_friction_Motor_l.motor_value->speed_rpm,3,1);
	oled_showstring(3,0,"o");
	oled_showfloat(3,3,hero_friction_Motor_l.Motor_PID_Speed.out,5,1);

//	oled_showstring(4,0,"te");
//	oled_shownum(4,2,IMU_Vale.temp,1,4);
}

void Oled_ShowGM3508Set_r(void)
{
	oled_showstring(0,0,"Kp");
	oled_showfloat(0,2,hero_friction_Motor_r.Motor_PID_Speed.Kp,3,1);
	oled_showstring(0,8,"Ki");
	oled_showfloat(0,10,hero_friction_Motor_r.Motor_PID_Speed.Ki,3,1);
	oled_showstring(1,0,"Kd");
	oled_showfloat(1,2,hero_friction_Motor_r.Motor_PID_Speed.Kd,4,1);

	oled_showstring(2,0,"t");
	oled_showfloat(2,2,hero_friction_Motor_r.motor_value->target_speed_dp10ms,3,1);
	oled_showstring(2,8,"n");
	oled_showfloat(2,10,hero_friction_Motor_r.motor_value->speed_dp10ms,3,1);
	oled_showstring(3,0,"o");
	oled_showfloat(3,3,hero_friction_Motor_r.Motor_PID_Speed.out,5,1);

//	oled_showstring(4,0,"te");
//	oled_shownum(4,2,IMU_Vale.temp,1,4);
}


void Oled_ShowMotor(void)
{
	oled_showstring(0,0,"1");
	oled_shownum(0,2,moto_CAN[0].speed_rpm,1,5);
	oled_showstring(1,0,"2");
	oled_shownum(1,2,moto_CAN[1].speed_rpm,1,5);
	oled_showstring(2,0,"3");
	oled_shownum(2,2,moto_CAN[2].speed_rpm,1,5);
	oled_showstring(3,0,"4");
	oled_shownum(3,2,moto_CAN[3].speed_rpm,1,5);
	oled_showstring(0,9,"5");
	oled_shownum(0,11,moto_CAN[4].speed_rpm,1,5);
	oled_showstring(1,9,"6");
	oled_shownum(1,11,moto_CAN[5].speed_rpm,1,5);
	oled_showstring(2,9,"7");
	oled_shownum(2,11,moto_CAN[6].speed_rpm,1,5);
	oled_showstring(3,9,"8");
	oled_shownum(3,11,moto_CAN[7].speed_rpm,1,5);

	oled_showstring(4,0,"9");
	oled_shownum(4,2,moto_CAN[8].speed_dp10ms,1,5);

	oled_showstring(4,9,"10");
	oled_shownum(4,12,moto_CAN[9].angle,1,5);
}

void Oled_ShowRC(void)
{
	oled_showstring(0,0,"ch1");
	oled_shownum(0,4,rc.ch1,1,3);
	oled_showstring(1,0,"ch2");
	oled_shownum(1,4,rc.ch2,1,3);
	oled_showstring(2,0,"ch3");
	oled_shownum(2,4,rc.ch3,1,3);
	oled_showstring(3,0,"ch4");
	oled_shownum(3,4,rc.ch4,1,3);
	oled_showstring(4,0,"sw1");
	oled_shownum(4,4,rc.sw1,1,3);
	oled_showstring(0,10,"sw2");
	oled_shownum(0,14,rc.sw2,1,3);
	oled_showstring(1,10,"whe");
	oled_shownum(1,14,rc.wheel,1,3);

}

void Oled_ShowRC_PC(void)
{
//	oled_showstring(0,1,"x");
//	oled_shownum(0,3,rc.x,1,3);
//	oled_showstring(1,1,"y");
//	oled_shownum(1,3,rc.y,1,3);
//	oled_showstring(2,1,"z");
//	oled_shownum(2,3,rc.z,1,3);
//	oled_showstring(3,1,"l");
//	oled_shownum(3,3,rc.l,1,3);
//	oled_showstring(4,1,"r");
//	oled_shownum(4,3,rc.r,1,3);

//	oled_showstring(0,7,"W");
//	oled_shownum(0,9,(rc.keyboard.bit.W),1,1);
//	oled_showstring(1,7,"S");
//	oled_shownum(1,9,(rc.keyboard.bit.S),1,1);
//	oled_showstring(2,7,"A");
//	oled_shownum(2,9,(rc.keyboard.bit.A),1,1);
//	oled_showstring(3,7,"D");
//	oled_shownum(3,9,(rc.keyboard.bit.D),1,1);
//	oled_showstring(4,7,"SH");
//	oled_shownum(4,9,(rc.keyboard.bit.SHIFT),1,1);

//	oled_showstring(0,11,"CT");
//	oled_shownum(0,13,(rc.keyboard.bit.CTRL),1,1);
//	oled_showstring(1,11,"Q");
//	oled_shownum(1,13,(rc.keyboard.bit.Q),1,1);
//	oled_showstring(2,11,"E");
//	oled_shownum(2,13,(rc.keyboard.bit.E),1,1);
//	oled_showstring(3,11,"R");
//	oled_shownum(3,13,(rc.keyboard.bit.R),1,1);
//	oled_showstring(4,11,"F");
//	oled_shownum(4,13,(rc.keyboard.bit.F),1,1);

//	oled_showstring(0,14,"G");
//	oled_shownum(0,16,(rc.keyboard.bit.G),1,1);
//	oled_showstring(1,14,"Z");
//	oled_shownum(1,16,(rc.keyboard.bit.Z),1,1);
//	oled_showstring(2,14,"X");
//	oled_shownum(2,16,(rc.keyboard.bit.X),1,1);
//	oled_showstring(3,14,"C");
//	oled_shownum(3,16,(rc.keyboard.bit.C),1,1);
//	oled_showstring(4,14,"V");
//	oled_shownum(4,16,(rc.keyboard.bit.V),1,1);
}

void Oled_ShowJY901(void)
{
	oled_showstring(0,0,"wx");
	oled_showfloat(0,2,JY901_Vale.GyroX,3,1);
	oled_showstring(0,8,"wy");
	oled_showfloat(0,10,JY901_Vale.GyroY,3,1);
	oled_showstring(1,0,"wz");
	oled_showfloat(1,2,JY901_Vale.GyroZ,4,1);

	oled_showstring(2,0,"Pi");
	oled_showfloat(2,2,JY901_Vale.Pitch,3,1);
	oled_showstring(2,8,"Ro");
	oled_showfloat(2,10,JY901_Vale.Roll,3,1);
	oled_showstring(3,0,"Ya");
	oled_showfloat(3,2,JY901_Vale.Yaw,4,1);
	oled_showstring(4,0,"Read");
	oled_shownum(4,8,ReadJY901Success,1,3);
}

void Oled_ShowMotorSet(void)
{
	oled_showstring(0,0,"1");
	oled_shownum(0,2,moto_CAN[0].target_speed_rpm,1,5);
	oled_showstring(1,0,"2");
	oled_shownum(1,2,moto_CAN[1].target_speed_rpm,1,5);
	oled_showstring(2,0,"3");
	oled_shownum(2,2,moto_CAN[2].target_speed_rpm,1,5);
	oled_showstring(3,0,"4");
	oled_shownum(3,2,moto_CAN[3].target_speed_rpm,1,5);
	oled_showstring(0,9,"5");
	oled_shownum(0,11,moto_CAN[4].target_speed_rpm,1,5);
	oled_showstring(1,9,"6");
	oled_shownum(1,11,moto_CAN[5].target_speed_rpm,1,5);
	oled_showstring(2,9,"7");
	oled_shownum(2,11,moto_CAN[6].target_speed_rpm,1,5);
	oled_showstring(3,9,"8");
	oled_shownum(3,11,moto_CAN[7].target_speed_rpm,1,5);

	oled_showstring(4,0,"9");
	oled_shownum(4,2,moto_CAN[8].target_speed_dp10ms,1,5);

	oled_showstring(4,9,"10");
	oled_shownum(4,12,moto_CAN[9].target_angle,1,5);

}
void oled_LOGO(void)
{
    oled_clear(Pen_Clear);
    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;
    for(; y < 64; y += 8)
    {
        for(x = 0; x < 128; x++)
        {
            temp_char = LOGO_BMP[x][y/8];
            for(i = 0; i < 8; i++)
            {
                if(temp_char & 0x80) oled_drawpoint(x, y + i,Pen_Write);
                else oled_drawpoint(x,y + i,Pen_Clear);
                temp_char <<= 1;
            }
        }
    }
    oled_refresh_gram();
    HAL_Delay(500);
}

