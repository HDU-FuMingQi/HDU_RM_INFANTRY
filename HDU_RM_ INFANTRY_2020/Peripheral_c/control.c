#include "control.h"

//����ģʽ
eRemoteMode remoteMode = RC;

//ϵͳ״̬
eSystemState systemState = SYSTEM_STARTING;


//���ؿ���ģʽ
eRemoteMode SYSTEM_GetRemoteMode( )
{
	return remoteMode;
}

//����ϵͳ״̬
eSystemState SYSTEM_GetSystemState( )
{
	return systemState;
}

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

