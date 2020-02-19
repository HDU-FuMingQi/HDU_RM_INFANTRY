#ifndef __BSP_CAN
#define __BSP_CAN


#include "stm32f4xx_hal.h"
#include "can.h"
#include "main.h"
#include "myType.h"
/*CAN发送或是接收的ID*/

typedef struct{
	int8_t InputVot;
	int8_t CapVot;
	int8_t Test_Current;
	int8_t Target_Power;
}power_measure_t;



/*接收到的云台电机的参数结构体*/



/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_CAN[];
extern moto_measure_t  moto_CAN2[];
extern power_measure_t PowerData;

void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr,uint8_t temp[]);
void get_chassis_power(power_measure_t *ptr,uint8_t temp[]);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto1234_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void set_moto5678_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void set_moto91011_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3);
/////////////////////////设置功率////////////////////
void set_supercap_current(CAN_HandleTypeDef* hcan, s16 iq1);

#endif
