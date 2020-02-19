/*
 * bsp_pid.c
 *
 *  Created on: 2019年10月18日
 *      Author: Tongw
 */
#include "bsp_pid.h"










#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    if(fabs(pid->Dead_Zone)<1e-5)
    	pid->Dead_Zone=0;
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_Calc(PidTypeDef *pid, const fp32 ref, const fp32 set)
{
	uint8_t index;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[0] = set - ref;

    if(pid->angle_max!=pid->angle_min)
    {
    	if( pid->error[0]>(pid->angle_max+pid->angle_min)/2)
    		 pid->error[0]-=(pid->angle_max+pid->angle_min);
    	else if( pid->error[0]<-(pid->angle_max+pid->angle_min)/2)
    		 pid->error[0]+=(pid->angle_max+pid->angle_min);
    }
    if(fabs(pid->error[0])<pid->Dead_Zone)	//死区
	{
		pid->error[0]=0;
	}
    if(fabs(pid->error[0])>pid->I_Separation)//误差过大，采用积分分离
    {
    	index=0;
    }
    else
    {
    	index=1;
    }
	pid->set = set;
	pid->fdb = ref;

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout=pid->Kd*(1- pid-> gama)*(pid->Dbuf[0])+pid-> gama* pid-> lastdout;
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + index*pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid-> lastdout=pid->Dout;
    return pid->out;
}


//fp32 PID_Calc_Angle(PidTypeDef *pid, fp32 ref, fp32 set)
//{
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }
//
//    pid->error[2] = pid->error[1];
//    pid->error[1] = pid->error[0];
//    pid->set = set;
//    pid->fdb = ref;
//    pid->error[0] = set - ref;
//    if(pid->error[0]>180)
//    {
//    	pid->error[0]-=360;
//    }
//    else if(pid->error[0]<-180)
//    {
//    	pid->error[0]+=360;
//    }
//    if(fabs(pid->error[0])<Dead_Zone)
//    {
//    	pid->error[0]=0;
//    }
//    if (pid->mode == PID_POSITION)
//    {
//        pid->Pout = pid->Kp * pid->error[0];
//        pid->Iout += pid->Ki * pid->error[0];
//        pid->Dbuf[2] = pid->Dbuf[1];
//        pid->Dbuf[1] = pid->Dbuf[0];
//        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
//        pid->Dout = pid->Kd * pid->Dbuf[0];
//        LimitMax(pid->Iout, pid->max_iout);
//        pid->out = pid->Pout + pid->Iout + pid->Dout;
//        LimitMax(pid->out, pid->max_out);
//    }
//    else if (pid->mode == PID_DELTA)
//    {
//        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
//        pid->Iout = pid->Ki * pid->error[0];
//        pid->Dbuf[2] = pid->Dbuf[1];
//        pid->Dbuf[1] = pid->Dbuf[0];
//        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
//        pid->Dout = pid->Kd * pid->Dbuf[0];
//        pid->out += pid->Pout + pid->Iout + pid->Dout;
//        LimitMax(pid->out, pid->max_out);
//    }
//    return pid->out;
//}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
