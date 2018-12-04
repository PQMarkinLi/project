/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_UP_Pin GPIO_PIN_2
#define LED_UP_GPIO_Port GPIOA
#define LED_UP_EXTI_IRQn EXTI2_IRQn
#define LED_DOWN_Pin GPIO_PIN_3
#define LED_DOWN_GPIO_Port GPIOA
#define LED_DOWN_EXTI_IRQn EXTI3_IRQn
#define ID1_Pin GPIO_PIN_4
#define ID1_GPIO_Port GPIOA
#define ID2_Pin GPIO_PIN_5
#define ID2_GPIO_Port GPIOA
#define ID3_Pin GPIO_PIN_6
#define ID3_GPIO_Port GPIOA
#define ID4_Pin GPIO_PIN_7
#define ID4_GPIO_Port GPIOA
#define KEY_UP_LED_Pin GPIO_PIN_1
#define KEY_UP_LED_GPIO_Port GPIOB
#define KEY_UP_LED_EXTI_IRQn EXTI0_IRQn
#define KEY_DOWN_LED_Pin GPIO_PIN_0
#define KEY_DOWN_LED_GPIO_Port GPIOB
#define KEY_DOWN_LED_EXTI_IRQn EXTI1_IRQn
#define KEY_DOWN_NO_Pin GPIO_PIN_12
#define KEY_DOWN_NO_GPIO_Port GPIOB
#define KEY_DOWN_NC_Pin GPIO_PIN_13
#define KEY_DOWN_NC_GPIO_Port GPIOB
#define KEY_UP_NO_Pin GPIO_PIN_14
#define KEY_UP_NO_GPIO_Port GPIOB
#define KEY_UP_NC_Pin GPIO_PIN_15
#define KEY_UP_NC_GPIO_Port GPIOB
#define RS485_DIR_Pin GPIO_PIN_8
#define RS485_DIR_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define RS485_DIR_L HAL_GPIO_WritePin(RS485_DIR_GPIO_Port,RS485_DIR_Pin,GPIO_PIN_RESET)
#define RS485_DIR_H HAL_GPIO_WritePin(RS485_DIR_GPIO_Port,RS485_DIR_Pin,GPIO_PIN_SET)
//
#define KEY_DOWN_NC_L HAL_GPIO_WritePin(KEY_DOWN_NC_GPIO_Port,KEY_DOWN_NC_Pin,GPIO_PIN_RESET)
#define KEY_DOWN_NC_H HAL_GPIO_WritePin(KEY_DOWN_NC_GPIO_Port,KEY_DOWN_NC_Pin,GPIO_PIN_SET)

#define KEY_DOWN_NO_L HAL_GPIO_WritePin(KEY_DOWN_NO_GPIO_Port,KEY_DOWN_NO_Pin,GPIO_PIN_RESET)
#define KEY_DOWN_NO_H HAL_GPIO_WritePin(KEY_DOWN_NO_GPIO_Port,KEY_DOWN_NO_Pin,GPIO_PIN_SET)

#define KEY_UP_NC_L HAL_GPIO_WritePin(KEY_UP_NC_GPIO_Port,KEY_UP_NC_Pin,GPIO_PIN_RESET)
#define KEY_UP_NC_H HAL_GPIO_WritePin(KEY_UP_NC_GPIO_Port,KEY_UP_NC_Pin,GPIO_PIN_SET)

#define KEY_UP_NO_L HAL_GPIO_WritePin(KEY_UP_NO_GPIO_Port,KEY_UP_NO_Pin,GPIO_PIN_RESET)
#define KEY_UP_NO_H HAL_GPIO_WritePin(KEY_UP_NO_GPIO_Port,KEY_UP_NO_Pin,GPIO_PIN_SET)


/* USER CODE END Private defines */
extern uint8_t up_state ;
extern uint8_t down_state ;
extern uint8_t running_state ;
extern uint8_t running_state_up ;
extern uint8_t running_state_down ;
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
