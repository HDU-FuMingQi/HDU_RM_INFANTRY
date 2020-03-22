#include "mysystem.h"
SYSTEMVALUE SystemValue=Starting;	//����״̬
EnvironmentModeType EnvironmentMode=NOMAL;	//��������
ControlModeType ControlMode=REMOTE;			//���Ʒ�ʽ
uint8_t ISVISIONAUTO =0 ;					//����ģʽ

/**
  * @brief  ��ʼ�����躯��
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_InitPeripheral(void)
{
	
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue, 30);//�򿪰���adc����
	oled_init();//oled��ʼ��
	dbus_uart_init();//ң������ʼ��
	jy901_uart_init();	//JY901��ʼ��
	vision_uart_init();	//�Ӿ�ͨ�ų�ʼ��
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);		//��������ʼ��
	HAL_UART_Receive_DMA(&huart3,Judge_Buffer,200);  ///�������ܲ���ϵͳ����/
	HAL_UART_Receive_IT(&huart3,Judge_Buffer,200);
	HAL_TIM_Base_Start(&htim6);       ////��������ͨ��CAN���Ʒ�����Ϣ����Ҫ��10HZ�Ķ�ʱ���ж��·���/
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim13);     /////����10ms�ж�����
	HAL_TIM_Base_Start_IT(&htim13);
										//���ݵ�����ȡ��ʼ��
										//���ݳ����Ƴ�ʼ��
										//����IO�ڳ�ʼ��
	
#if (SAFE_MODE==1)
	while(SystemValue==Starting&&rc.sw1!=1)		//���������ʱ������С����ģʽ
	{
		HAL_Delay(1);	
		htim12.Init.Period =rc.ch2*4+9999;		//�ı䶨ʱ��Ƶ��
		HAL_TIM_PWM_Init(&htim12);
		TIM12->CCR1=abs(rc.ch3)*10;				//�ı䶨ʱ��ռ�ձ�
	}
#endif
	//��can1�����ڵ���Ŀ���
	  my_can_filter_init_recv_all(&hcan1);
	  HAL_CAN_Start(&hcan1);
	  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	  my_can_filter_init_recv_all(&hcan2);
	  HAL_CAN_Start(&hcan2);
	  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	//��/�رյ����Դ���
	  HAL_GPIO_WritePin(POWER1_GPIO_Port,POWER1_Pin,1);	//24V 1
	  HAL_GPIO_WritePin(POWER2_GPIO_Port,POWER2_Pin,1); //24V 2
	  HAL_GPIO_WritePin(POWER3_GPIO_Port,POWER3_Pin,1); //24V 3
	  HAL_GPIO_WritePin(POWER4_GPIO_Port,POWER4_Pin,1); //24V 4
	  HAL_GPIO_WritePin(POWER5_GPIO_Port,POWER5_Pin,1); //24V 4
		
	TIM12->CCR1=0;
	ICM20602_Init();
	
	//SystemValue=Running;	
}

//������ʼ��
void Parameter_Init(void)
{
	CHASSIS_InitArgument();//���̲�����ʼ��
	GIMBAL_InitArgument();//��̨������ʼ��
}

/**
  * @brief  ��������������
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

		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		count++;
		count%=25;
		if(count==24)
			LED1_Toggle;
#if (SAFE_MODE==1)
		if(currentTime>REMOTE_ulGetLostTime()||REMOTE_IfDataError()==true)	//ң����ʧ��ʱ�����,�������ݳ���
		{
			vTaskSuspend(GimbalTaskHandle);		//���������
			vTaskSuspend(ChassisTaskHandle);
			vTaskSuspend(ShootTaskHandle);
			
			vTaskResume(OutControl_TaskHandle);//���ʧ�ر�����������
			
			TIM12->CCR1=5000;
		}
		else
		{
			vTaskResume(GimbalTaskHandle);		//�ָ�����
			vTaskResume(ChassisTaskHandle);
			vTaskResume(ShootTaskHandle);
			HAL_GPIO_WritePin(POWER5_GPIO_Port,POWER5_Pin,1); //24V 4
			vTaskSuspend(OutControl_TaskHandle);//����ʧ�ر�����������
			TIM12->CCR1=0;
		}
#endif
		vTaskDelayUntil(&currentTime, 40);//������ʱ
	}
}

//ʧ�ؿ��������������ÿ4msִ��һ�Σ�����ӳ�
void OutControl_Fun(void *pvParameters)
{
	for(;;)
	{	
		vTaskDelay(TIME_STAMP_4MS);				//4ms
		
		SystemValue=Starting;//ϵͳ�ָ�������״̬
		REMOTE_vResetData();//ң�����ݻָ���Ĭ��״̬
		
		//�ر����е�����
		set_moto1234_current(&hcan1,0,0,0,0);
		set_moto5678_current(&hcan1,0,0,0,0);
		HAL_GPIO_WritePin(POWER5_GPIO_Port,POWER5_Pin,0); //24V 4
	}
}
