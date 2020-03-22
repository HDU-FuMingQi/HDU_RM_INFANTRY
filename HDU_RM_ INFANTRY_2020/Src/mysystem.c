#include "mysystem.h"
SYSTEMVALUE SystemValue=Starting;	//程序状态
EnvironmentModeType EnvironmentMode=NOMAL;	//所处环境
ControlModeType ControlMode=REMOTE;			//控制方式
uint8_t ISVISIONAUTO =0 ;					//自瞄模式

/**
  * @brief  初始化外设函数
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_InitPeripheral(void)
{
	
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue, 30);//打开按键adc传输
	oled_init();//oled初始化
	dbus_uart_init();//遥控器初始化
	jy901_uart_init();	//JY901初始化
	vision_uart_init();	//视觉通信初始化
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);		//蜂鸣器初始化
	HAL_UART_Receive_DMA(&huart3,Judge_Buffer,200);  ///开启接受裁判系统数据/
	HAL_UART_Receive_IT(&huart3,Judge_Buffer,200);
	HAL_TIM_Base_Start(&htim6);       ////超级电容通过CAN控制发送信息但需要在10HZ的定时器中断下发送/
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim13);     /////发射10ms中断启动
	HAL_TIM_Base_Start_IT(&htim13);
										//电容电量读取初始化
										//电容充电控制初始化
										//电容IO口初始化
	
#if (SAFE_MODE==1)
	while(SystemValue==Starting&&rc.sw1!=1)		//程序刚启动时不能在小陀螺模式
	{
		HAL_Delay(1);	
		htim12.Init.Period =rc.ch2*4+9999;		//改变定时器频率
		HAL_TIM_PWM_Init(&htim12);
		TIM12->CCR1=abs(rc.ch3)*10;				//改变定时器占空比
	}
#endif
	//打开can1，用于电机的控制
	  my_can_filter_init_recv_all(&hcan1);
	  HAL_CAN_Start(&hcan1);
	  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	  my_can_filter_init_recv_all(&hcan2);
	  HAL_CAN_Start(&hcan2);
	  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	//打开/关闭电机电源输出
	  HAL_GPIO_WritePin(POWER1_GPIO_Port,POWER1_Pin,1);	//24V 1
	  HAL_GPIO_WritePin(POWER2_GPIO_Port,POWER2_Pin,1); //24V 2
	  HAL_GPIO_WritePin(POWER3_GPIO_Port,POWER3_Pin,1); //24V 3
	  HAL_GPIO_WritePin(POWER4_GPIO_Port,POWER4_Pin,1); //24V 4
	  HAL_GPIO_WritePin(POWER5_GPIO_Port,POWER5_Pin,1); //24V 4
		
	TIM12->CCR1=0;
	ICM20602_Init();
	
	//SystemValue=Running;	
}

//参数初始化
void Parameter_Init(void)
{
	CHASSIS_InitArgument();//底盘参数初始化
	GIMBAL_InitArgument();//云台参数初始化
}

/**
  * @brief  检测程序正常运行
  * @param  void
  * @retval void
  * @attention 
  */
void ShowRunningFun(void const * argument)
{
	portTickType currentTime;	
	static uint32_t count=0;
	for(;;)
	{

		currentTime = xTaskGetTickCount();//当前系统时间
		count++;
		count%=25;
		if(count==24)
			LED1_Toggle;
#if (SAFE_MODE==1)
		if(currentTime>REMOTE_ulGetLostTime()||REMOTE_IfDataError()==true)	//遥控器失联时间过长,或者数据出错
		{
			vTaskSuspend(GimbalTaskHandle);		//将任务挂起
			vTaskSuspend(ChassisTaskHandle);
			vTaskSuspend(ShootTaskHandle);
			
			vTaskResume(OutControl_TaskHandle);//解挂失控保护控制任务
			
			TIM12->CCR1=5000;
		}
		else
		{
			vTaskResume(GimbalTaskHandle);		//恢复任务
			vTaskResume(ChassisTaskHandle);
			vTaskResume(ShootTaskHandle);
			HAL_GPIO_WritePin(POWER5_GPIO_Port,POWER5_Pin,1); //24V 4
			vTaskSuspend(OutControl_TaskHandle);//挂起失控保护控制任务
			TIM12->CCR1=0;
		}
#endif
		vTaskDelayUntil(&currentTime, 40);//绝对延时
	}
}

//失控控制任务若解挂则每4ms执行一次，相对延迟
void OutControl_Fun(void *pvParameters)
{
	for(;;)
	{	
		vTaskDelay(TIME_STAMP_4MS);				//4ms
		
		SystemValue=Starting;//系统恢复至重启状态
		REMOTE_vResetData();//遥控数据恢复至默认状态
		
		//关闭所有电机输出
		set_moto1234_current(&hcan1,0,0,0,0);
		set_moto5678_current(&hcan1,0,0,0,0);
		HAL_GPIO_WritePin(POWER5_GPIO_Port,POWER5_Pin,0); //24V 4
	}
}
