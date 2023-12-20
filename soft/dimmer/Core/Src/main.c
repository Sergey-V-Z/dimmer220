/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TIME_OUT_DS18B20_Read 1000
//#define TIME_OUT_DIMMER 1000
#define TIME_OUT_DIMMER_PULSE 1

#define PERIOD_US 10000		// 10 ms
#define DELAY_PULSE_US 5000 // 5 ms
#define FAN_0 10000;		// 0 %
#define FAN_P1 5000;		// 50 %
#define FAN_P2 4000;		// 60 %
#define FAN_P3 3000;		// 70 %
#define FAN_P4 2000;		// 80 %
#define FAN_P5 5;		// 100 %
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flag_pulse_Start = 0;

float Temperature = 0.0;
uint8_t Presence = 0;
uint8_t Temp_byte1, Temp_byte2;
uint16_t TEMP;
float setTEMP = 25.0;
float stepTemp = 3.0;

uint32_t oldTime = 0;
uint32_t time = 0;

uint32_t time_DS18B20 = 0;
uint32_t time_out_DS18B20 = 1;
uint32_t time_DS18B20_Read = 0;
uint8_t event_DS18B20 = 0;

uint32_t time_Dimmer = 0;
//uint32_t time_Dimmer_Pulse = 0;
uint32_t time_out_dimmer = 20; // 20 mS не включается семистор 0 включен постоянно
uint8_t event_dimmer = 0;
uint32_t delay_dimm_us = 5000; // 10000 us = 10 ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay(uint16_t time) {
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim16)) < time)
		;
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(TX_1W_GPIO_Port, TX_1W_Pin);   // set the pin as output
	HAL_GPIO_WritePin (TX_1W_GPIO_Port, TX_1W_Pin, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(TX_1W_GPIO_Port, TX_1W_Pin);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (TX_1W_GPIO_Port, TX_1W_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(TX_1W_GPIO_Port, TX_1W_Pin);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(TX_1W_GPIO_Port, TX_1W_Pin);  // set as output
			HAL_GPIO_WritePin (TX_1W_GPIO_Port, TX_1W_Pin, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(TX_1W_GPIO_Port, TX_1W_Pin);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(TX_1W_GPIO_Port, TX_1W_Pin);
			HAL_GPIO_WritePin (TX_1W_GPIO_Port, TX_1W_Pin, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(TX_1W_GPIO_Port, TX_1W_Pin);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(TX_1W_GPIO_Port, TX_1W_Pin);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(TX_1W_GPIO_Port, TX_1W_Pin);   // set as output

		HAL_GPIO_WritePin (TX_1W_GPIO_Port, TX_1W_Pin, 0);  // pull the data pin LOW
		delay (1);  // wait for > 1us

		Set_Pin_Input(TX_1W_GPIO_Port, TX_1W_Pin);  // set as input
		delay (15);
		if (HAL_GPIO_ReadPin (TX_1W_GPIO_Port, TX_1W_Pin))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (50);  // wait for 60 us
	}
	return value;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	oldTime = HAL_GetTick();
	HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// чтение температуры по таймауту
		if (time_DS18B20_Read >= TIME_OUT_DS18B20_Read) {
			switch (event_DS18B20) {
			case 0:
				Presence = DS18B20_Start();
				time_out_DS18B20 = 1; // установки таймера задержки
				time_DS18B20 = 0;
				event_DS18B20 = 1;
				break;
			case 1:
				if (time_DS18B20 >= time_out_DS18B20) {
					DS18B20_Write(0xCC);  // skip ROM
					DS18B20_Write(0x44);  // convert t
					time_out_DS18B20 = 800; // установки таймера задержки
					time_DS18B20 = 0;
					event_DS18B20 = 2;
				}
				break;
			case 2:
				if (time_DS18B20 >= time_out_DS18B20) {
					Presence = DS18B20_Start();
					time_out_DS18B20 = 1; // установки таймера задержки
					time_DS18B20 = 0;
					event_DS18B20 = 3;
				}
				break;
			case 3:
				if (time_DS18B20 >= time_out_DS18B20) {
					DS18B20_Write(0xCC);  // skip ROM
					DS18B20_Write(0xBE);  // Read Scratch-pad

					Temp_byte1 = DS18B20_Read();
					Temp_byte2 = DS18B20_Read();
					TEMP = (Temp_byte2 << 8) | Temp_byte1;
					Temperature = (float) TEMP / 16;

					time_out_DS18B20 = 0;	// сброс таймера задержки
					time_DS18B20 = 0;
					time_DS18B20_Read = 0; // сброс таймера опроса
					event_DS18B20 = 0;
				}
				break;
			default:
				break;
			}
		}

		// расчитать таймер димера в зависимости от температуры


		// управление симистором
		if(Temperature >= setTEMP) // 25
		{
			if(Temperature >= (setTEMP + stepTemp)) //28
			{
				if(Temperature >= (setTEMP + stepTemp * 2)) //31
				{

					if(Temperature >= (setTEMP + stepTemp * 3)) // 34
					{
						if(Temperature >= (setTEMP + stepTemp * 4)) //37
						{
							delay_dimm_us = FAN_P5;
						}
						else // от 34 до 37
						{
							delay_dimm_us = FAN_P4;
						}
					}
					else // от 31 до 34
					{
						delay_dimm_us = FAN_P3;
					}
				}
				else // от 28 до 31
				{
					delay_dimm_us = FAN_P2;
				}
			}
			else // от 25 до 28
			{
				delay_dimm_us = FAN_P1;
			}
		}
		else
		{
			delay_dimm_us = FAN_0;
		}

		/*if (flag_Dimer_Start) {
			switch (event_dimmer) {
			case 0:
				if (time_Dimmer >= time_out_dimmer) {
					//HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_SET);
					event_dimmer = 1;
					time_Dimmer = 0;
				}

				break;
			case 1:
				if (time_Dimmer >= TIME_OUT_DIMMER_PULSE) {
					//HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_RESET);
					event_dimmer = 0;
					time_Dimmer = 0;
					flag_Dimer_Start = 0;
				}
				break;
			default:
				break;
			}
		}
		*/



		if ((HAL_GetTick() - oldTime) > 1) {

			time = (HAL_GetTick() - oldTime);

			time_DS18B20_Read += time;
			time_DS18B20 += time;

			/*if (flag_Dimer_Start) {
				time_Dimmer += time;
			}*/
			time = 0;
			oldTime = HAL_GetTick();
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ZC_INT_Pin) {
		HAL_GPIO_TogglePin(B2_GPIO_Port, B2_Pin);
		HAL_TIM_Base_Stop_IT(&htim17);
		__HAL_TIM_SET_AUTORELOAD(&htim17, delay_dimm_us);// таймар на открытие симистора
		__HAL_TIM_SET_COUNTER(&htim17, 0);
		//HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_RESET);
		flag_pulse_Start = 0;
		HAL_TIM_Base_Start_IT(&htim17); // запуск таймара на открытие симистора
	} else {
		__NOP();
	}
}

//HAL_TIM_OC_Start_IT(&htim17);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(B1_GPIO_Port, B1_Pin);

	if(flag_pulse_Start) // если импульс запущен то остановим таймер
	{
		flag_pulse_Start = 0;
		HAL_TIM_Base_Stop_IT(&htim17);
		HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_AUTORELOAD(&htim17, 0);
		__HAL_TIM_SET_COUNTER(&htim17, 0);

	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim17);
		__HAL_TIM_SET_AUTORELOAD(&htim17, DELAY_PULSE_US);// таймар на длинну ипульса
		__HAL_TIM_SET_COUNTER(&htim17, 0);
		HAL_TIM_Base_Start_IT(&htim17); // запуск таймара на длинну ипульса
		HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
		flag_pulse_Start = 1;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
