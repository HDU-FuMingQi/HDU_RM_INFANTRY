#include "control.h"


float abs_float(float a)
{
	if(a<0)
		return -a;
	else 
		return a;
}


float constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

		if (*buffer > 0)
		{
				if (*buffer > ramp)
				{  
						now     += ramp;
					  *buffer -= ramp;
				}   
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		else
		{
				if (*buffer < -ramp)
				{
						now     += -ramp;
					  *buffer -= -ramp;
				}
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		
		return now;
}


void LimtValue_f(float* VALUE,float MAX,float MIN)
{
	if(*VALUE>MAX)
		*VALUE=MAX;
	else if(*VALUE<MIN)
		*VALUE=MIN;
}

void LimtValue_u16(uint16_t* VALUE,uint16_t MAX,uint16_t MIN)
{
	if(*VALUE>MAX)
		*VALUE=MAX;
	else if(*VALUE<MIN)
		*VALUE=MIN;
}

void LimtValue_16(int16_t* VALUE,int16_t MAX,int16_t MIN)
{
	if(*VALUE>MAX)
		*VALUE=MAX;
	else if(*VALUE<MIN)
		*VALUE=MIN;
}

/****************�Ƕ����ƺ���*******************/
/**
  * @brief  �ǶȻػ� ����
  * @param  void
  * @retval �Ƕ�ֵ�����Ƕ�ֵ
  * @attention 
  */
void AngleLoop_f (float* angle ,float max)
{
	if(*angle<-(max/2))
	{
		*angle+=max;
	}
	else if(*angle>(max/2))
	{
		*angle-=max;
	}
}
/**
  * @brief  �ǶȻػ� 
  * @param  void
  * @retval �Ƕ�ֵ�����Ƕ�ֵ
  * @attention 
  */
void AngleLoop (float* angle ,float max)
{
	if(*angle<-(max/2))
	{
		*angle+=max;
	}
	else if(*angle>(max/2))
	{
		*angle-=max;
	}
}

