#ifndef __IWDG_H__
#define __IWDG_H__

#include "stm32f1xx_hal.h"
#include "main.h"
extern IWDG_HandleTypeDef IWDG_Handler;


void IWDG_init(uint8_t prer,uint16_t rlr);
void IWDG_Feed(void);
#endif
