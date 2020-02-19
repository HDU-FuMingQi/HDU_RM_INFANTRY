/*
 * jy901.c
 *
 *  Created on: Nov 21, 2019
 *      Author: Tongw
 */

#include "jy901.h"
#include "bsp_dbus.h"
struct _IMU_Vale JY901_Vale=
{
	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,0
};
uint8_t   jy901_buf[JY901_BUFLEN];
uint8_t   ReadJY901Success=0;	//0:未接收数据 1：成功接收数据 255：接收错误数据
void jy901_uart_init(void)
{
	/* open uart idle it */


	uart_receive_dma_no_it(&JY901_HUART, jy901_buf, JY901_MAX_LEN);

	__HAL_UART_CLEAR_IDLEFLAG(&JY901_HUART);
	__HAL_UART_ENABLE_IT(&JY901_HUART, UART_IT_IDLE);
}
void jy901_callback_handler(uint8_t *buff)
{
	if(buff[0]==0x55||buff[11]==0x55)
	{
		switch(buff[1])
		{
		case 0x52:
//			JY901_Vale.GyroX=(float)((int16_t)buff[3]<<8|(int16_t)buff[2])/32768*2000;	//°/s
			JY901_Vale.GyroX=(float)((int8_t)buff[3]<<8|buff[2])/32768*2000;
			JY901_Vale.GyroY=(float)((int8_t)buff[5]<<8|buff[4])/32768*2000;	//°/s
			JY901_Vale.GyroZ=(float)((int8_t)buff[7]<<8|buff[6])/32768*2000;	//°/s
			ReadJY901Success=1;
			break;
		case 0x53:
			JY901_Vale.Roll= (float)((int8_t)buff[3]<<8|buff[2])/32768*180;	//°
			JY901_Vale.Pitch=(float)((int8_t)buff[5]<<8|buff[4])/32768*180;	//°
			JY901_Vale.Yaw=  (float)((int8_t)buff[7]<<8|buff[6])/32768*180;	//°
			ReadJY901Success=2;
			break;
		default:
			ReadJY901Success=255;
		}
		switch(buff[12])
		{
		case 0x52:
			JY901_Vale.GyroX=(float)((int8_t)buff[14]<<8|buff[13])/32768*2000;	//°/s
			JY901_Vale.GyroY=(float)((int8_t)buff[16]<<8|buff[15])/32768*2000;	//°/s
			JY901_Vale.GyroZ=(float)((int8_t)buff[18]<<8|buff[17])/32768*2000;	//°/s
			ReadJY901Success=3;
			break;
		case 0x53:
			JY901_Vale.Roll= (float)((int8_t)buff[14]<<8|buff[13])/32768*180;	//°
			JY901_Vale.Pitch=(float)((int8_t)buff[16]<<8|buff[15])/32768*180;	//°
			JY901_Vale.Yaw=  (float)((int8_t)buff[18]<<8|buff[17])/32768*180;	//°
			ReadJY901Success=4;
			break;
		default:
			ReadJY901Success=254;
		}
	}
	else
	{
		ReadJY901Success=254;
		return ;
	}
}



