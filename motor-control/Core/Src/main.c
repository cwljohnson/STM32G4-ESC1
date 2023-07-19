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
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t adc1_dma[2];
volatile uint16_t adc2_dma[2];
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_OPAMP3_Init();
  MX_OPAMP2_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  /* USER CODE BEGIN 2 */
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);

	while (LL_ADC_IsCalibrationOnGoing(ADC1)) {
	};
	while (LL_ADC_IsCalibrationOnGoing(ADC2)) {
	};

	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);

	LL_OPAMP_Enable(OPAMP1);
	LL_OPAMP_Enable(OPAMP2);
	LL_OPAMP_Enable(OPAMP3);

	LL_mDelay(1000);

	LL_TIM_EnableCounter(TIM1);

	LL_TIM_GenerateEvent_UPDATE(TIM1);

	LL_ADC_REG_StartConversion(ADC1);
	LL_ADC_REG_StartConversion(ADC2);

	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR, (uint32_t)adc1_dma,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&ADC2->DR, (uint32_t)adc2_dma,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

	LL_GPIO_SetOutputPin(P1_L_GPIO_Port, P1_L_Pin);
	LL_GPIO_SetOutputPin(P2_L_GPIO_Port, P2_L_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	char buffer[200];
	  uint8_t num = 0;

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  LL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

		  uint32_t bus_mV = (adc1_dma[0] * 3300UL) >> 12;
		  bus_mV = (bus_mV * (169+18)) / 18;

		  int32_t shunt_mV[3];
		  shunt_mV[0] = 0;//(adc1_dma[0] * 3300UL) >> 12;
		  shunt_mV[1] = (adc2_dma[0] * 3300UL) >> 12;
		  shunt_mV[2] = 0;//(adc2_dma[1] * 3300UL) >> 12;

		  float shuntMv1 = (((adc2_dma[0] * 3300.0f) / 4096.0f) - 2048.0f) * 577.0f / 16.0f;

		  shunt_mV[1] = (int32_t)shuntMv1; //((shunt_mV[1] - 2048) * 577) / 16; //((shunt_mV[1] - 2048) * 577) / 16;

		  num = sprintf(buffer, "B %4i S %4i %4i %4i\r\n", bus_mV, shunt_mV[0], shunt_mV[1], shunt_mV[2]);
		  uart_tx_bytes_blocking((uint8_t*)buffer, num);



		LL_mDelay(100);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 30, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(120000000);

  LL_SetSystemCoreClock(120000000);
}

/* USER CODE BEGIN 4 */

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
