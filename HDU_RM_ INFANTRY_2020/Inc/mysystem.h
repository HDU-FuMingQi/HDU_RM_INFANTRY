#ifndef MYSYSTEM_H
#define MYSYSTEM_H
#include "main.h"

extern osThreadId ShowRunningTaskHandle;
extern osThreadId ShowOLEDTaskHandle;
extern osThreadId GimbalTaskHandle;
extern osThreadId ChassisTaskHandle;
extern osThreadId ShootTaskHandle;
extern osThreadId OutControl_TaskHandle;


#define LED2(n) (HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,n))
#define LED1(n) (HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,n))
#define LED3(n) (HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,n))
#define LED2_Toggle (HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin))
#define LED1_Toggle (HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin))
#define LED3_Toggle (HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin))
#define SAFE_MODE 1

#define    TIME_STAMP_1MS        1
#define    TIME_STAMP_2MS        2
#define    TIME_STAMP_4MS        4
#define    TIME_STAMP_10MS      10
#define    TIME_STAMP_20MS      20
#define    TIME_STAMP_30MS      30
#define    TIME_STAMP_40MS      40
#define    TIME_STAMP_50MS      50
#define    TIME_STAMP_60MS      60
#define    TIME_STAMP_80MS      80
#define    TIME_STAMP_100MS    100
#define    TIME_STAMP_150MS    150
#define    TIME_STAMP_200MS    200
#define    TIME_STAMP_250MS    250
#define    TIME_STAMP_300MS    300
#define    TIME_STAMP_400MS    400
#define    TIME_STAMP_500MS    500
#define    TIME_STAMP_1000MS  1000
#define    TIME_STAMP_2000MS  2000
#define    TIME_STAMP_10S     10000

#define    FALSE    0
#define    TRUE     1
typedef enum
{
	NOMAL,		//普通模式
	CLIMBING,	//爬坡模式
}EnvironmentModeType;
typedef enum
{
	KEYBOARD,
	REMOTE,
}ControlModeType;
extern ControlModeType ControlMode;
extern EnvironmentModeType EnvironmentMode;
extern SYSTEMVALUE SystemValue;	//程序状态
extern uint8_t ISVISIONAUTO;	
void SYSTEM_InitPeripheral(void);
void LimtValue_f(float* VALUE,float MAX,float MIN);
void LimtValue_16(int16_t* VALUE,int16_t MAX,int16_t MIN);
void LimtValue_u16(uint16_t* VALUE,uint16_t MAX,uint16_t MIN);
//////////斜坡函数,使目标输出值缓慢等于指针输入值///////////////
float RampInc_float( float *buffer, float now, float ramp );
#endif


