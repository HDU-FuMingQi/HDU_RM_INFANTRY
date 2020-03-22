#ifndef __bspdbus_h
#define __bspdbus_h
#include "main.h"
#include "usart.h"
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */
/**
  * @brief  remote control information
  */
	
typedef   signed          char    bool_t;
 
 /* 遥控器通信协议 
   当接收机和发射机建立连接后，接收机会每隔 7ms 通过DBUS发送一帧
   数据（18 字节） */
#define    REMOTE_DBUS_FRAME_LEN    18


/* ----------------------- RC Channel Definition---------------------------- */

#define    RC_CH_VALUE_MIN       ((int16_t)-660 )
#define    RC_CH_VALUE_OFFSET    ((int16_t)0)
#define    RC_CH_VALUE_MAX       ((int16_t)660)


/* ----------------------- RC Switch Definition----------------------------- */

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)
  
/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)


typedef struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;

  uint8_t sw1;
  uint8_t sw2;
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
		}mouse;
		
		struct
		{
			uint16_t v;
		}key;
    int16_t wheel;
} rc_info_t;

/* 获取遥控器摇杆偏移量
   根据遥控器文档：

左摇杆：    右摇杆：
左右为ch2   左右为ch0
上下为ch3   上下为ch1

                        上   660           
左    中       右       
-660   0      660       中   0

                        下  -660  */


/* 获取遥控器摇杆偏移值 
   RLR：右摇杆左右移动  LUD：左摇杆上下移动	*/
#define    RC_CH0_RLR_OFFSET    (rc.ch0 - RC_CH_VALUE_OFFSET)
#define    RC_CH1_RUD_OFFSET  	(rc.ch1 - RC_CH_VALUE_OFFSET)
#define    RC_CH2_LLR_OFFSET  	(rc.ch2 - RC_CH_VALUE_OFFSET)
#define    RC_CH3_LUD_OFFSET  	(rc.ch3 - RC_CH_VALUE_OFFSET)


/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (rc.sw1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc.sw1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc.sw1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc.sw2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc.sw2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc.sw2 == RC_SW_DOWN)


/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (rc.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (rc.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (rc.mouse.z)


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc.mouse.press_r == 1)


/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  rc.key.v  )
#define    IF_KEY_PRESSED_W       ( (rc.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )



extern uint8_t   dbus_buf[DBUS_BUFLEN];
extern rc_info_t rc;
void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
extern portTickType ulRemoteLostTime ;
/* 获取遥控器失联倒计时 */
uint32_t REMOTE_ulGetLostTime( void );
/* 刷新失联时间 */
void REMOTE_vUpdateLostTime( void );
bool REMOTE_IfDataError( void );
void REMOTE_vResetData( void );
#endif

  //鼠标
//  int16_t x;
//  int16_t y;
//  int16_t z;

//  uint8_t l;
//  uint8_t r;

//  //键盘
//  union {
//      uint16_t key_code;
//      struct
//      {
//        uint16_t W : 1;
//        uint16_t S : 1;
//        uint16_t A : 1;
//        uint16_t D : 1;
//        uint16_t SHIFT : 1;
//        uint16_t CTRL : 1;
//        uint16_t Q : 1;
//        uint16_t E : 1;
//        uint16_t R : 1;
//        uint16_t F : 1;
//        uint16_t G : 1;
//        uint16_t Z : 1;
//        uint16_t X : 1;
//        uint16_t C : 1;
//        uint16_t V : 1;
//        uint16_t B : 1;
//      } bit;
//    } keyboard;