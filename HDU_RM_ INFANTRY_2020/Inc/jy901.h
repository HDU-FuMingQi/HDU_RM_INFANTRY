#ifndef jy901_h
#define jy901_h
#include "main.h"
void jy901_uart_init(void);
#define JY901_HUART huart8
#define JY901_BUFLEN 22
#define JY901_MAX_LEN 40
void jy901_callback_handler(uint8_t *buff);
extern uint8_t   jy901_buf[];
extern struct _IMU_Vale JY901_Vale;
extern uint8_t   ReadJY901Success;
#endif
