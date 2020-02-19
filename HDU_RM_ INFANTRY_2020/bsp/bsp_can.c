/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "can.h"
#include "bsp_can.h"
#include "bsp_oled.h"
#include "stm32f4xx_hal_can.h"

moto_measure_t moto_CAN[11] ={0};
moto_measure_t moto_CAN2[11] = {0};
power_measure_t PowerData = {0};//1 supercap

void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		sFilterConfig;


   sFilterConfig.FilterIdHigh         = 0x0000;
   sFilterConfig.FilterIdLow          = 0x0000;
   sFilterConfig.FilterMaskIdHigh     = 0x0000;
   sFilterConfig.FilterMaskIdLow      = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterBank=27;
   sFilterConfig.FilterScale=CAN_FILTERSCALE_16BIT;
   sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
	{

	}


}


/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
uint8_t text;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{
	CAN_RxHeaderTypeDef _RxHeader;
	uint8_t temp[8];

	HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&_RxHeader,temp);

	//ignore can1 or can2.
	switch(_RxHeader.StdId){
		case CAN_Moto1_ID:
		case CAN_Moto2_ID:
		case CAN_Moto3_ID:
		case CAN_Moto4_ID:
		case CAN_Moto5_ID:
		case CAN_Moto6_ID:
		case CAN_Moto7_ID:
		case CAN_Moto8_ID:
		case CAN_Moto9_ID:
		case CAN_Moto10_ID:
		case CAN_Moto11_ID:
			{
				static u8 i;
				i = _RxHeader.StdId - CAN_Moto1_ID;
				if(_hcan==&hcan1)
					get_moto_measure(&moto_CAN[i],temp);
				else
					get_moto_measure(&moto_CAN2[i], temp);
			}
			break;
		case 0x211:
		{
			get_chassis_power(&PowerData,temp);
		}
		break;
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/

	  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*******************************************************************************************
  * @Func			void get_chassis_power(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收底盘通过CAN发过来的信息（用买来的充电电容进行充电）
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/


void get_chassis_power(power_measure_t *ptr,uint8_t temp[])
{
	ptr->InputVot=(uint16_t)(temp[0] | temp[1]<<8 ) /100.f;
	ptr->CapVot=(uint16_t)(temp[2] | temp[3]<<8 ) /100.f;
	ptr->Test_Current=(uint16_t)(temp[4] | temp[5]<<8 ) /100.f ;
	ptr->Target_Power=(uint16_t)(temp[6] | temp[7]<<8 ) /100.f;
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr,uint8_t temp[])
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(temp[0]<<8 | temp[1]) ;
	ptr->speed_rpm  = (int16_t)(temp[2]<<8 |temp[3]);
	ptr->real_current = (temp[4]<<8 | temp[5])*5.f/16384.f;
	//ptr->real_current = (temp[4]<<8 | temp[5]);

	ptr->hall = temp[6];


	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = 0;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){

	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void set_moto1234_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x200;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (iq1 >> 8);
	Txtemp[1] = iq1;
	Txtemp[2] = (iq2 >> 8);
	Txtemp[3] = iq2;
	Txtemp[4] = iq3 >> 8;
	Txtemp[5] = iq3;
	Txtemp[6] = iq4 >> 8;
	Txtemp[7] = iq4;
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {
		
	  }

}

void set_moto5678_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x1FF;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (iq1 >> 8);
	Txtemp[1] = iq1;
	Txtemp[2] = (iq2 >> 8);
	Txtemp[3] = iq2;
	Txtemp[4] = iq3 >> 8;
	Txtemp[5] = iq3;
	Txtemp[6] = iq4 >> 8;
	Txtemp[7] = iq4;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0);
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {

	  }

}

void set_moto91011_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3 ){
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x2FF;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (iq1 >> 8);
	Txtemp[1] = iq1;
	Txtemp[2] = (iq2 >> 8);
	Txtemp[3] = iq2;
	Txtemp[4] = iq3 >> 8;
	Txtemp[5] = iq3;
//	Txtemp[6] = iq4 >> 8;
//	Txtemp[7] = iq4;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {

	  }

}

void set_supercap_current(CAN_HandleTypeDef* hcan, s16 iq1)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t sendbuf[8];
	_TxHeader.StdId = 0x210;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	sendbuf[0] = (iq1 >> 8);
	sendbuf[1] = iq1;
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,sendbuf,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{

	}
}

