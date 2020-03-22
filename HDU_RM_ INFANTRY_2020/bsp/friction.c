#include "friction.h"


/*****************����Ƶ���뽵�����٣����������Ħ�������***********************/

//���ļ��кܶ಻̫����ĵط�,ֻ��Ϊ��������

//���ļ�����10ms��ʱ���жϷ�����
 
/**************************Ħ���ֿ���***********************************/


//Ħ���ֲ�ͬpwm�¶�Ӧ����������ֵ(����),��ñ�ʵ��ֵ��5
uint16_t Friction_PWM_HeatInc[5] = {0,  20,  26,  34,  36};//����ʱ��㶨���ٶ�,������Ը���


//�ٶȵȼ�ѡ��
uint16_t Fric_Speed_Level;

//Ħ����ʵ������ٶ�,������б�����
float Friction_Speed_Real;


/************************Ħ�����ܿ���*****************************/
//Ħ�����ٶ�ѡ��,Ӱ���ӵ��ٶ�,ֻ�ǲ���,ϵ��Ҫ���涨
float Friction_PWM_Output[6]     = {0, 460, 505, 583, 695, 685};//�ر�  ����  ����  ����  ��  �ڱ�

//�ٶȵȼ�ѡ��
uint16_t Fric_Speed_Level;

//ң��ģʽ�µĿ�����־λ
uint8_t Friction_Switch = 0;//�뵯�ֿ����ж�����

//Ħ����Ŀ���ٶ�
float Friction_Speed_Target;

//Ħ���ֵȼ�Ŀ��ת��
float Frict_Speed_Level_Target;

//Ħ����ʵ������ٶ�,������б�����
float Friction_Speed_Real;

//�ٶ�ѡ��
#define FRI_OFF  	0
#define FRI_LOW  	1		//Z������
#define FRI_MID  	2		//B������
#define FRI_HIGH 	3		//���
#define FRI_MAD  	4		//�������
#define FRI_SENTRY  5		//�ڱ�����

/**
  * @brief  Ħ���ֿ��ƺ���
  * @param  void
  * @retval void
  * @attention Ħ����ֹͣת��,����ر�,Ħ���ִӹرյ�����,��̨̧ͷ����
  */
void FRICTION_Ctrl()
{
	if(SystemValue==Starting)
	{
		Fric_Speed_Level = FRI_OFF;
		Friction_Speed_Target = 0;
		Friction_Speed_Real   = 0;
	}
	else
	{
		if(ControlMode==REMOTE)
		{
		}
		else if(ControlMode==KEYBOARD)
		{
			FRIC_KeyLevel_Ctrl();
		}
	}
	
	//Ħ�������б��,ע��Ҫ��̧ͷ���ܽ���б��
	Friction_Ramp();
	
//	//���⿪��
//	if(Friction_Speed_Real > 0 && GIMBAL_IfBuffHit() == FALSE && Magazine_IfOpen() == FALSE)//��ʵ���������һ��
//	{
//		Laser_On;
//	}
//	else
//	{
//		Laser_Off;
//		//Laser_On;
//	}
	
	//���÷����ٶ�
	TIM_FrictionPwmOutp(Friction_Speed_Target,Friction_Speed_Target);
}

/**
  * @brief  Ħ���ֵȼ�����,����ר��,���ݵȼ��Զ���������
  * @param  ��ǰ�ȼ�
  * @retval void
  * @attention ����ģʽ�²���Ħ����
  */
float debug_fric = 450;//760;//650;
float Fric_Dist_Far = 3;//���ڴ˾�����������
float Fric_Dist_Near = 1.2;//С�ڴ˾����������
void FRIC_KeyLevel_Ctrl(void)
{
	static int16_t fric_auto_delay = 0;//��ֹ����Ƶ������
	float fric_percent = 1;
	float Fric_Dist_Now = 5;
	
	Fric_Dist_Now = VisionValue.distance/100;\
	
	if (GIMBAL_IfBuffHit( ) == TRUE)
	{
		Fric_Speed_Level = FRI_MAD;//���ģʽ,�������
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//����ѡ��
	}
	else if (IF_MOUSE_PRESSED_LEFT && GIMBAL_IfAutoHit() == FALSE)//�����飬�������
	{
		Fric_Speed_Level = FRI_HIGH;//һ���ø�����
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//����ѡ��
	}
	else if (IF_KEY_PRESSED_Z)//�Ƽ�����
	{
		Fric_Speed_Level = FRI_LOW;				
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//����ѡ��		
	}
	else if (IF_KEY_PRESSED_B)
	{
		Fric_Speed_Level = FRI_MID;//����Ƶ������ģʽ,�Ƽ��ⲫ��
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//����ѡ��
	}
		
	else if (GIMBAL_IfAutoHit() == TRUE)//����ģʽ
	{
		if(VisionValue.identify_target == TRUE)
		{
			fric_auto_delay = 0;
			
			fric_percent = (Fric_Dist_Now - Fric_Dist_Near) / (Fric_Dist_Far - Fric_Dist_Near);
			fric_percent = constrain_float(fric_percent, 0, 1);
			
			Frict_Speed_Level_Target = Friction_PWM_Output[FRI_LOW] 
										+ (Friction_PWM_Output[FRI_HIGH] - Friction_PWM_Output[FRI_LOW]) * fric_percent;
		}
		else//ûʶ�𵽣���һ��ʱ������л��ٶ�
		{
			fric_auto_delay++;
			if(fric_auto_delay >= 20)//����200msûʶ��
			{
				fric_auto_delay = 200;//��ֹ���
				Fric_Speed_Level = FRI_HIGH;//һ���ø�����
				Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//����ѡ��
			}
		}
		///////������ڱ�
		if(GIMBAL_AUTO_PITCH_SB() == TRUE)//������ڱ����������
		{
			Fric_Speed_Level = FRI_SENTRY;
			Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//����ѡ��
		}
	}
	
	else//��ֹ����
	{
		Fric_Speed_Level = FRI_HIGH;	
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//����ѡ��
	}

	if( IF_MOUSE_PRESSED_LEFT || Friction_Speed_Target>0 )//�����м����������BUG����̧ͷ���Զ���ǹ
	{
		//Friction_Speed_Target = debug_fric;
		Friction_Speed_Target = Frict_Speed_Level_Target;
	}
}


/**
  * @brief  ��ǰPWM��Ӧ�����ӵ���������ֵ(����)
  * @param  void
  * @retval ��ǰ��������ֵ
  * @attention ��������42mm,�����ڿͻ���������ʾ
  */
uint16_t Fric_GetHeatInc(void)
{
	if(GIMBAL_IfAutoHit() == TRUE && !IF_KEY_PRESSED_Z && !IF_KEY_PRESSED_B)
	{
		return JUDGE_usGetSpeedHeat17()*1.4f;
	}
	else
	{
		return Friction_PWM_HeatInc[Fric_Speed_Level];
	}
}

/**
  * @brief  ��ȡ��ǰĦ����PWM���ֵ
  * @param  void
  * @retval ʵ��PWMֵ
  * @attention ������ֹĦ�����ٶȹ��͵�����²��̵�ת��
  */
float Fric_GetSpeedReal(void)
{
	return Friction_Speed_Real;
}

/*************Ħ���ָ�������****************/

/**
  * @brief  Ħ�������б��
  * @param  void
  * @retval void
  * @attention 
  */
void Friction_Ramp(void)
{
	if (Friction_Speed_Real < Friction_Speed_Target)//����
	{
		Friction_Speed_Real += 5;
		if(Friction_Speed_Real > Friction_Speed_Target)
		{
			Friction_Speed_Real = Friction_Speed_Target;
		}
	}
	else if (Friction_Speed_Real > Friction_Speed_Target)//�ر�
	{
		Friction_Speed_Real -= 5;
	}
	
	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}
}

/**
  * @brief  Ħ�����������
  * @param  void
  * @retval pwm1  pwm2
  * @attention ���Ҷ�����,�Ժ���ߵ�ʱ��ע�������ߵĽӷ�
  */
void TIM_FrictionPwmOutp(int16_t pwm1,int16_t pwm2)//8,9
{
	htim2.Instance->CCR1= pwm1+1000;  //�������ʱҪ����1ms
	htim2.Instance->CCR2= pwm2+1000;
}













