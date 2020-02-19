/*
 * bsp_dbus.c
 *
 *  Created on: Oct 16, 2019
 *      Author: Tongw
 */
#include "bsp_dbus.h"
#include "jy901.h"

uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc;
/* 超过这个时间没有收到新的遥控器数据就认为已经失控 */
#define    REMOTE_LOST_TIME    ((uint32_t)50)   //50ms
portTickType ulRemoteLostTime = 0;
/* 获取遥控器失联倒计时 */
uint32_t REMOTE_ulGetLostTime( void )
{
	/* */
	return  ulRemoteLostTime;
}
/* 刷新失联时间 */
void REMOTE_vUpdateLostTime( void )
{
	/* 当接收到数据时，刷新失联倒计时 */
	ulRemoteLostTime = xTaskGetTickCount( ) + REMOTE_LOST_TIME;
}

/**
  * @brief      enable global uart it and do not use DMA transfer done it
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
	  uint32_t tmp = 0;

	  tmp = huart->RxState;
	  if (tmp == HAL_UART_STATE_READY)
	  {
	    if ((pData == NULL) || (Size == 0))
	    {
	      return HAL_ERROR;
	    }

	    /* Process Locked */
	    __HAL_LOCK(huart);

	    huart->pRxBuffPtr = pData;
	    huart->RxXferSize = Size;

	    huart->ErrorCode = HAL_UART_ERROR_NONE;

	    /* Enable the DMA Stream */
	    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
	                  (uint32_t)pData, Size);

	    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
	    in the UART CR3 register */
	    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	    /* Process Unlocked */
	    __HAL_UNLOCK(huart);

	    return HAL_OK;
	  }
	  else
	  {
	    return HAL_BUSY;
	  }
}

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}



/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval
  */
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{

	  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	  rc->ch1 -= 1024;
	  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	  rc->ch2 -= 1024;
	  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	  rc->ch3 -= 1024;
	  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	  rc->ch4 -= 1024;

	  /* prevent remote control zero deviation */
	  if(rc->ch1 <= 5 && rc->ch1 >= -5)
	    rc->ch1 = 0;
	  if(rc->ch2 <= 5 && rc->ch2 >= -5)
	    rc->ch2 = 0;
	  if(rc->ch3 <= 5 && rc->ch3 >= -5)
	    rc->ch3 = 0;
	  if(rc->ch4 <= 5 && rc->ch4 >= -5)
	    rc->ch4 = 0;

	  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	  rc->sw2 = (buff[5] >> 4) & 0x0003;

	  //接受出现问题
	  if ((abs(rc->ch1) > 660) ||
	      (abs(rc->ch2) > 660) ||
	      (abs(rc->ch3) > 660) ||
	      (abs(rc->ch4) > 660))
	  {
	    memset(rc, 0, sizeof(rc));
	    return ;
	  }

	  rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
	  rc->mouse.y = buff[8] | (buff[9] << 8);
	  rc->mouse.z = buff[10] | (buff[11] << 8);

	  rc->mouse.press_l = buff[12];
	  rc->mouse.press_r = buff[13];

	  rc->key.v = buff[14] | buff[15] << 8; // key borad code
	  rc->wheel = (buff[16] | buff[17] << 8) - 1024;
	  return;
}

//遥控数据混乱
bool REMOTE_IfDataError( void )
{
	if ( (rc.sw1 != RC_SW_UP && rc.sw1 != RC_SW_MID && rc.sw1 != RC_SW_DOWN)
		|| (rc.sw2 != RC_SW_UP && rc.sw2 != RC_SW_MID && rc.sw2 != RC_SW_DOWN)
		|| (rc.ch4 > RC_CH_VALUE_MAX || rc.ch4 < RC_CH_VALUE_MIN)
		|| (rc.ch1 > RC_CH_VALUE_MAX || rc.ch1 < RC_CH_VALUE_MIN)
		|| (rc.ch2 > RC_CH_VALUE_MAX || rc.ch2 < RC_CH_VALUE_MIN)
		|| (rc.ch3 > RC_CH_VALUE_MAX || rc.ch3 < RC_CH_VALUE_MIN) )
	{
		return true;
	}
	else
	{
		return false;
	}
}


/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* handle received data in idle interrupt */
	if (huart == &DBUS_HUART)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		//判断读到的数据是不是18个字节，如果是十八个字节进入回调函数解析
//		ReadJY901Success=huart->hdmarx->Instance->NDTR;
		
		if ((DBUS_MAX_LEN - huart->hdmarx->Instance->NDTR) == DBUS_BUFLEN)
		{
			REMOTE_vUpdateLostTime();
			rc_callback_handler(&rc, dbus_buf);
		}

		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);

	}
	else if (huart == &JY901_HUART)
	{
//		ReadJY901Success=5;
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		//判断读到的数据是不是18个字节，如果是十八个字节进入回调函数解析

		if ((JY901_MAX_LEN - huart->hdmarx->Instance->NDTR) == JY901_BUFLEN)
		{
			jy901_callback_handler(jy901_buf);
		}

		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, JY901_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
	else if (huart == &VISION_HUART)
	{
//		ReadJY901Success=5;
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		//判断读到的数据是不是18个字节，如果是十八个字节进入回调函数解析
		ReadJY901Success=VISION_MAX_LEN - huart->hdmarx->Instance->NDTR;
		if ((VISION_MAX_LEN - huart->hdmarx->Instance->NDTR) == VISION_BUFLEN)
		{
			vision_callback_handler(vision_buf);
			Vision_Get_New_Data = TRUE;//标记视觉数据更新了
		}

		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, VISION_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
  * @brief      callback this function when uart interrupt
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{

		uart_rx_idle_callback(huart);
	}
}

/**
  * @brief   initialize dbus uart device
  * @param
  * @retval
  */
void dbus_uart_init(void)
{
	/* open uart idle it */


	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);

	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
}
/* 复位数据 */
void REMOTE_vResetData( void )
{
	/* Channel 0, 1, 2, 3 */
	rc.ch1 = RC_CH_VALUE_OFFSET;
	rc.ch2 = RC_CH_VALUE_OFFSET;
	rc.ch3 = RC_CH_VALUE_OFFSET;
	rc.ch4 = RC_CH_VALUE_OFFSET;

	/* Switch left, right */
	rc.sw1  = RC_SW_MID;
	rc.sw2  = RC_SW_MID;

	/* Mouse axis: X, Y, Z */
//	rc.x = 0;
//	rc.y = 0;
//	rc.z = 0;

	/* Mouse Left, Right Is Press ? */
//	rc.l = 0;
//	rc.r = 0;

	/* KeyBoard value */
//	rc.keyboard.key_code= 0;
}

