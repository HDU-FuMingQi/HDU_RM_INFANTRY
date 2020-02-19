#include "control.h"

//控制模式
eRemoteMode remoteMode = RC;

//系统状态
eSystemState systemState = SYSTEM_STARTING;


//返回控制模式
eRemoteMode SYSTEM_GetRemoteMode( )
{
	return remoteMode;
}

//返回系统状态
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

