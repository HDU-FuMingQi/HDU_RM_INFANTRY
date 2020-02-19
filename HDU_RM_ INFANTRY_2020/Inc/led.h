#ifndef _ledh
#define _ledh
#include "main.h"

#define LED2(n) (HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,n))
#define LED1(n) (HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,n))
#define LED2_Toggle (HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin))
#define LED1_Toggle (HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin))
#endif
