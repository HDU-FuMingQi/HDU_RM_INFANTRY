#ifndef _CONTROL_H
#define _CONTROL_H

#include "main.h"

typedef enum
{
    RC   = 0,  
    KEY  = 1,  

} eRemoteMode;  // 遥控方式


typedef enum
{
	  SYSTEM_STARTING  = 0,
	  SYSTEM_RUNNING   = 1,

} eSystemState;




//控制
void SYSTEM_Reset( void );
void SYSTEM_OutCtrlProtect( void );
void SYSTEM_UpdateSystemState( void );
void SYSTEM_UpdateRemoteMode( void );
eRemoteMode SYSTEM_GetRemoteMode( void );
eSystemState SYSTEM_GetSystemState( void );
//float类型绝对值函数
float abs_float(float a);

float constrain_float(float amt, float low, float high);

#endif


