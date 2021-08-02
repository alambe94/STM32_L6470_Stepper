/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "L6470.h"
#include "stdlib.h"
#include "math.h"
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
/* Private variables ---------------------------------------------------------*/
StepperMotorDriverHandle_t Motor_X_Handler;
StepperMotorDriverHandle_t Motor_Y_Handler;
StepperMotorDriverHandle_t Motor_Z_Handler;
StepperMotorDriverHandle_t Motor_M_Handler;

MotorParameterData_t Motor_1_Data;
MotorParameterData_t Motor_X_Data;
MotorParameterData_t Motor_Y_Data;
MotorParameterData_t Motor_Z_Data;
MotorParameterData_t Motor_M_Data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void)
{
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Motor_1_Data.motorvoltage = 9.0;
  Motor_1_Data.fullstepsperrevolution = 200;
  Motor_1_Data.phasecurrent = 1.5;
  Motor_1_Data.phasevoltage = 3.0;
  Motor_1_Data.speed = 100.0;
  Motor_1_Data.acc = 500.0;
  Motor_1_Data.dec = 500.0;
  Motor_1_Data.maxspeed = 1000.0;
  Motor_1_Data.minspeed = 5.0;
  Motor_1_Data.fsspd = 602.7;
  Motor_1_Data.kvalhold = 3.06;
  Motor_1_Data.kvalrun = 3.06;
  Motor_1_Data.kvalacc = 3.06;
  Motor_1_Data.kvaldec = 3.06;
  Motor_1_Data.intspeed = 61.52;
  Motor_1_Data.stslp = 392.1569e-6;
  Motor_1_Data.fnslpacc = 643.1372e-6;
  Motor_1_Data.fnslpdec = 643.1372e-6;
  Motor_1_Data.kterm = 0;
  Motor_1_Data.ocdth = 1 * 1500 * 1.00;
  Motor_1_Data.stallth = 1000 * 1.00;
  Motor_1_Data.step_sel = MICROSTEP_1_128;
  Motor_1_Data.alarmen = 0xFF;
  Motor_1_Data.config = 0x2E88;

  Motor_X_Data.motorvoltage = 9.0;
  Motor_X_Data.fullstepsperrevolution = 200;
  Motor_X_Data.phasecurrent = 1.5;
  Motor_X_Data.phasevoltage = 3.0;
  Motor_X_Data.speed = 100.0;
  Motor_X_Data.acc = 100.0;
  Motor_X_Data.dec = 50.0;
  Motor_X_Data.maxspeed = 1000.0;
  Motor_X_Data.minspeed = 0.0;
  Motor_X_Data.fsspd = 602.7;
  Motor_X_Data.kvalhold = 3.06;
  Motor_X_Data.kvalrun = 3.06;
  Motor_X_Data.kvalacc = 3.06;
  Motor_X_Data.kvaldec = 3.06;
  Motor_X_Data.intspeed = 61.52;
  Motor_X_Data.stslp = 392.1569e-6;
  Motor_X_Data.fnslpacc = 643.1372e-6;
  Motor_X_Data.fnslpdec = 643.1372e-6;
  Motor_X_Data.kterm = 0;
  Motor_X_Data.ocdth = 1 * 1500 * 1.00;
  Motor_X_Data.stallth = 1000 * 1.00;
  Motor_X_Data.step_sel = MICROSTEP_1_128;
  Motor_X_Data.alarmen = 0xFF;
  Motor_X_Data.config = 0x2E88;

  Motor_Y_Data.motorvoltage = 9.0;
  Motor_Y_Data.fullstepsperrevolution = 200;
  Motor_Y_Data.phasecurrent = 1.5;
  Motor_Y_Data.phasevoltage = 3.0;
  Motor_Y_Data.speed = 100.0;
  Motor_Y_Data.acc = 100.0;
  Motor_Y_Data.dec = 50.0;
  Motor_Y_Data.maxspeed = 1000.0;
  Motor_Y_Data.minspeed = 0.0;
  Motor_Y_Data.fsspd = 602.7;
  Motor_Y_Data.kvalhold = 3.06;
  Motor_Y_Data.kvalrun = 3.06;
  Motor_Y_Data.kvalacc = 3.06;
  Motor_Y_Data.kvaldec = 3.06;
  Motor_Y_Data.intspeed = 61.52;
  Motor_Y_Data.stslp = 392.1569e-6;
  Motor_Y_Data.fnslpacc = 643.1372e-6;
  Motor_Y_Data.fnslpdec = 643.1372e-6;
  Motor_Y_Data.kterm = 0;
  Motor_Y_Data.ocdth = 1 * 1500 * 1.00;
  Motor_Y_Data.stallth = 1000 * 1.00;
  Motor_Y_Data.step_sel = MICROSTEP_1_128;
  Motor_Y_Data.alarmen = 0xFF;
  Motor_Y_Data.config = 0x2E88;

  L6470_DISABLE();
  HAL_Delay(10);
  L6470_ENABLE();

  L6470_ResetDevice(0);
  L6470_GetStatus(0);

  L6470_ResetDevice(1);
  L6470_GetStatus(1);

  L6470_ResetDevice(2);
  L6470_GetStatus(2);

  L6470_ResetDevice(3);
  L6470_GetStatus(3);

  Motor_X_Handler.DaisyChainPosition = 0;
  Motor_X_Handler.Command = &L6470Command;
  L6470_Config(&Motor_X_Handler, &Motor_X_Data);

  Motor_Y_Handler.DaisyChainPosition = 1;
  Motor_Y_Handler.Command = &L6470Command;
  L6470_Config(&Motor_Y_Handler, &Motor_1_Data);

  Motor_Z_Handler.DaisyChainPosition = 2;
  Motor_Z_Handler.Command = &L6470Command;
  L6470_Config(&Motor_Z_Handler, &Motor_1_Data);

  Motor_M_Handler.DaisyChainPosition = 3;
  Motor_M_Handler.Command = &L6470Command;
  L6470_Config(&Motor_M_Handler, &Motor_1_Data);

  L6470_Run(1, L6470_DIR_FWD_ID, 1000);

  L6470_Run(1, L6470_DIR_REV_ID, 2500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
/*
    if (GPIO_Pin == L6470_Flag_INT_Pin)
	{
	uint16_t status_register_0 = L6470_GetStatus(0);
	uint16_t status_register_1 = L6470_GetStatus(1);
	uint16_t status_register_2 = L6470_GetStatus(2);
	uint16_t status_register_3 = L6470_GetStatus(3);

	if ((status_register_0 & STATUS_STEP_LOSS_A) == 0
		|| (status_register_0 & STATUS_STEP_LOSS_B) == 0)
	    {
	    L6470_HardStop(0);
	    }
	if ((status_register_1 & STATUS_STEP_LOSS_A) == 0
		|| (status_register_1 & STATUS_STEP_LOSS_B) == 0)
	    {
	    L6470_HardStop(1);
	    }
	if ((status_register_2 & STATUS_STEP_LOSS_A) == 0
		|| (status_register_2 & STATUS_STEP_LOSS_B) == 0)
	    {
	    L6470_HardStop(2);
	    }
	if ((status_register_3 & STATUS_STEP_LOSS_A) == 0
		|| (status_register_3 & STATUS_STEP_LOSS_B) == 0)
	    {
	    L6470_HardStop(3);
	    }

	}
*/
    if (GPIO_Pin == L6470_Flag_INT_Pin)
	{
	uint16_t status_register_0 = L6470_GetStatus(0);
	uint16_t status_register_1 = L6470_GetStatus(1);
	uint16_t status_register_2 = L6470_GetStatus(2);
	uint16_t status_register_3 = L6470_GetStatus(3);

	uint8_t perform_action = 0;

	if ((status_register_0 & STATUS_SW_EVN))
	    {
	    perform_action = 1;
	    L6470_PrepareReleaseSW(0, L6470_ACT_RST_ID, L6470_DIR_REV_ID);
	    }
	if ((status_register_1 & STATUS_SW_EVN))
	    {
	    perform_action = 1;
	    L6470_PrepareReleaseSW(1, L6470_ACT_RST_ID, L6470_DIR_FWD_ID);
	    }
	if ((status_register_2 & STATUS_SW_EVN))
	    {
	    perform_action = 1;
	    L6470_PrepareReleaseSW(2, L6470_ACT_RST_ID, L6470_DIR_REV_ID);
	    }
	if ((status_register_3 & STATUS_SW_EVN))
	    {
	    perform_action = 1;
	    L6470_PrepareReleaseSW(3, L6470_ACT_RST_ID, L6470_DIR_REV_ID);
	    }

	if (perform_action)
	    {
	    L6470_PerformPreparedApplicationCommand();
	    }

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
  while(1)
  {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
