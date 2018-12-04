#include "iwdg.h"

IWDG_HandleTypeDef IWDG_Handler;
// Tout = ((4*2^prer)*rlr)/32 
void IWDG_init(uint8_t prer,uint16_t rlr)
{
	
	IWDG_Handler.Instance = IWDG;
	IWDG_Handler.Init.Prescaler = prer;
	IWDG_Handler.Init.Reload = rlr;
	HAL_IWDG_Init(&IWDG_Handler);
	
	
	__HAL_IWDG_START(&IWDG_Handler);
}

void IWDG_Feed(void)
{
	HAL_IWDG_Refresh(&IWDG_Handler);
}
