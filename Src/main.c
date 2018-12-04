/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define TIMES (100)
#define MIN_TIMES (10)
#define MAX_TIMES (TIMES-10)
#define FLOOR_DOWN 0xf0
#define FLOOR_UP   0x0f
typedef enum{
	RUNNING = 0x01,
	ARRIVED = 0x02,
	STATIC = 0x03
}RUNNINGTYPE;
uint8_t bus_id;
uint8_t up_h_times=0,up_l_times=0,up_s_times=0;
uint8_t down_h_times=0,down_l_times=0,down_s_times = 0;
uint8_t running_state =0;
uint8_t running_state_up =0;
uint8_t running_state_down =0;
uint8_t floor_c = 0;
uint32_t time_ms = 0;
uint32_t time_500ms = 0;
uint8_t timeout_flag = 0;
uint8_t up_state = 0;
uint8_t down_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void down_led_dec(void);
void up_led_dec(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
GPIO_PinState pre_up_led_state ;
GPIO_PinState up_led_state ;
GPIO_PinState down_led_state ;
GPIO_PinState pre_down_led_state ;
int up_times = 0,down_times = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	//uint8_t state_data
	
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
		// Tout = ((4*2^prer)*rlr)/32  
	// Tout = IWDG_PRESCALER_256 * rlr /32
	//8 S
	IWDG_init(IWDG_PRESCALER_64,1000);
  /* ��ʼ������ */
  //NCΪ��
  KEY_DOWN_NC_L;KEY_UP_NC_L;
  KEY_DOWN_NO_L;KEY_UP_NO_L;


// ����ʱ��2
	
	HAL_TIM_Base_Start_IT(&htim2);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,UsartType1.usartDMA_rxBuf,RECEIVELEN);

  /*  */
	//printf("id:%d\n",bus_id);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	up_led_state = HAL_GPIO_ReadPin(LED_UP_GPIO_Port,LED_UP_Pin);
	pre_up_led_state = up_led_state;
	
	down_led_state = HAL_GPIO_ReadPin(LED_DOWN_GPIO_Port,LED_DOWN_Pin);
	pre_down_led_state = down_led_state;
  while (1)
  {
  /* ��ȡBUSID */

  bus_id = get_id();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  //LED io���

		IWDG_Feed();
		if(HAL_GPIO_ReadPin(KEY_UP_LED_GPIO_Port,KEY_UP_LED_Pin) != GPIO_PIN_RESET)
		{
			up_state = 0;
		}
		else
		{
			up_state = 1;
		}
		if(HAL_GPIO_ReadPin(KEY_DOWN_LED_GPIO_Port,KEY_DOWN_LED_Pin) != GPIO_PIN_RESET)
		{
			down_state = 0;
		}
		else
		{
			down_state = 1;
		}
		down_led_dec();
		up_led_dec();


 //if(running_state_up == STATIC || running_state_down == STATIC )
  {
    if(rs485_data == FLOOR_DOWN && running_state_down == STATIC)
		{
			if( HAL_GPIO_ReadPin(KEY_DOWN_LED_GPIO_Port,KEY_DOWN_LED_Pin) != GPIO_PIN_RESET )		
			{
				//���°���
				{
					KEY_DOWN_NO_H;
					time_ms = 0;
				}
			}
		}
		if(rs485_data == FLOOR_UP && running_state_up == STATIC)
		{
			if( HAL_GPIO_ReadPin(KEY_UP_LED_GPIO_Port,KEY_UP_LED_Pin) != GPIO_PIN_RESET )		
				{
					//���°���
					{
						KEY_UP_NO_H;
						time_ms = 0;
					}
				}
		}
		rs485_data = 0;
  }
	// 时间参数 
	HAL_Delay(15);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
	RS485_DIR_H;
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	RS485_DIR_L;
	return ch;
}
void down_led_dec(void)
{
		down_times++;
		down_led_state = HAL_GPIO_ReadPin(LED_DOWN_GPIO_Port,LED_DOWN_Pin);
		if(pre_down_led_state != down_led_state)
		{
			down_s_times++;
			pre_down_led_state = down_led_state;
		}
		if(down_led_state == GPIO_PIN_RESET)
		{
			down_l_times++;
		}
		else
		{
			down_h_times++;
		}
			if( down_s_times > 2 && down_l_times > 30 && down_h_times > 30)
			{
				running_state_down = ARRIVED;
			}
			else if(down_l_times==0 && down_h_times > (TIMES-10))
			{
				running_state_down = STATIC;
			}
			else if(down_l_times>(TIMES-10) && down_h_times ==0)
			{
				running_state_down = RUNNING;
			}
		if(down_times > TIMES )
		{

			down_times = 0;
			down_h_times = 0;
			down_l_times = 0;
			down_s_times = 0;
		}	
}
void up_led_dec(void)
{
		up_times++;
		up_led_state = HAL_GPIO_ReadPin(LED_UP_GPIO_Port,LED_UP_Pin);
		if(pre_up_led_state != up_led_state)
		{
			up_s_times++;
			pre_up_led_state = up_led_state;
		}
		if(up_led_state == GPIO_PIN_RESET)
		{
			up_l_times++;
		}
		else
		{
			up_h_times++;
		}
			if( up_s_times > 2 && up_l_times > 30 && up_h_times > 30)
			{
				running_state_up = ARRIVED;
			}
			else if(up_l_times==0 && up_h_times > (TIMES-10))
			{
				running_state_up = STATIC;
			}
			else if(up_l_times>(TIMES-10) && up_h_times ==0)
			{
				running_state_up = RUNNING;
			}
		if(up_times > TIMES )
		{
			up_times = 0;
			up_h_times = 0;
			up_l_times = 0;
			up_s_times = 0;
		}	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
