
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "L6470.h"
#include "ring_buffer.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
StepperMotorDriverHandle_t Motor_1_Handler;
StepperMotorDriverHandle_t Motor_2_Handler;
StepperMotorDriverHandle_t Motor_3_Handler;
StepperMotorDriverHandle_t Motor_4_Handler;



//MotorParameterData_t Motor_1_Data;
MotorParameterData_t Motor_1_Data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Motor_2_Home(void)
{
	L6470_SetParam(2,L6470_MAX_SPEED_ID,Step_s_2_MaxSpeed(1400));
	L6470_SetParam(2,L6470_ACC_ID,Step_s2_2_Acc(1200));
	L6470_SetParam(2,L6470_DEC_ID,Step_s2_2_Dec(1200));
	L6470_SetParam(2,L6470_STALL_TH_ID,mA_2_StallTh(800));
    L6470_Move(2,L6470_DIR_FWD_ID,200*64*100);
    while(HAL_GPIO_ReadPin(L6470_Flag_INT_GPIO_Port,L6470_Flag_INT_Pin) == GPIO_PIN_SET);

    uint16_t StatusRegister = L6470_GetStatus(2);

    if((StatusRegister & STATUS_BUSY) == 0)
      {
        //HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET);
      }

    if((StatusRegister & STATUS_STEP_LOSS_A) == 0 || (StatusRegister & STATUS_STEP_LOSS_B) == 0)
      {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET);
        L6470_HardStop(2);
      }
}


void UART_Loop()
    {

    uint8_t str_to_int[15] = "0";
    uint32_t rx_value[4] =
	{
	0
	};
    uint8_t motor_id = 0;
    uint8_t rx_digit_cnt = 0;
    uint8_t rx_byte = 0;

    uint8_t command_valid = 0;

    if (Ring_Buffer_Get_Count() > 0)
	{

	if (Ring_Buffer_Check_Char() == '\n') //complete command received

	    {

	    uint8_t while_loop_timeout_1 = 128; //equal to ring buffer size

	    while (rx_byte != '\n' && --while_loop_timeout_1)
		{

		SCAN_AXIS: rx_byte = Ring_Buffer_Get_Char();

		if (rx_byte == 'X' || rx_byte == 'x')
		    {
		    motor_id = 0;
		    goto PARSE_AXIS_VALUE;
		    }
		else if (rx_byte == 'Y' || rx_byte == 'y')
		    {
		    motor_id = 1;
		    goto PARSE_AXIS_VALUE;
		    }
		else if (rx_byte == 'Z' || rx_byte == 'z')
		    {
		    motor_id = 2;
		    goto PARSE_AXIS_VALUE;
		    }
		else if (rx_byte == 'M' || rx_byte == 'm')
		    {
		    motor_id = 3;
		    goto PARSE_AXIS_VALUE;
		    }
		else
		    {
		    goto SKIP;
		    //skip parsing
		    }

		PARSE_AXIS_VALUE: command_valid = 1;

		rx_byte = Ring_Buffer_Get_Char();

		if (rx_byte == 32) //if space
		    {
		    rx_byte = Ring_Buffer_Get_Char(); //Ignore space
		    }

		rx_digit_cnt = 0;

		uint8_t while_loop_timeout_2 = 128; //equal to ring bugger size

		while (rx_byte != '\n' && --while_loop_timeout_2)
		    {

		    if (rx_byte > 47 && rx_byte < 58) //if number
			{
			str_to_int[rx_digit_cnt++] = rx_byte;
			}
		    else if (rx_byte == 32) // space found - scan for next axis
			{
			str_to_int[rx_digit_cnt++] = '\n'; // close string
			rx_value[motor_id] = atoi((char*) str_to_int);
			goto SCAN_AXIS;
			}
		    else if (rx_byte != 13) // Carriage return ignore
			{
			command_valid = 0;
			}

		    rx_byte = Ring_Buffer_Get_Char();

		    }

		SKIP:
		    {
		    }

		}

	    if (command_valid == 1)
		{

		HAL_UART_Transmit(&huart2, (uint8_t*) "OK\n", 3, 2);

		str_to_int[rx_digit_cnt++] = '\n'; // close string
		rx_value[motor_id] = atoi((char*) str_to_int);

		if (rx_value[0] != 0)
		    {
		    L6470_Move(0, L6470_DIR_FWD_ID, rx_value[0]);
		    }
		if (rx_value[1] != 0)
		    {
		    L6470_Move(1, L6470_DIR_FWD_ID, rx_value[1]);
		    }
		if (rx_value[2] != 0)
		    {
		    L6470_Move(2, L6470_DIR_FWD_ID, rx_value[2]);
		    }
		if (rx_value[3] != 0)
		    {
		    L6470_Move(3, L6470_DIR_FWD_ID, rx_value[3]);
		    }

		}
	    else
		{
		HAL_UART_Transmit(&huart2, (uint8_t*) "Invalid Command\n", 16,
			2);
		}

	    }

	}

    }


void HAL_SYSTICK_Callback(void)
    {
    }



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  Motor_1_Data.motorvoltage=9.0;
  Motor_1_Data.fullstepsperrevolution=200;
  Motor_1_Data.phasecurrent=1.5;
  Motor_1_Data.phasevoltage=3.0;
  Motor_1_Data.speed=1000.0;
  Motor_1_Data.acc=1000.0;
  Motor_1_Data.dec=1000.0;
  Motor_1_Data.maxspeed=1500.0;
  Motor_1_Data.minspeed=0.0;
  Motor_1_Data.fsspd=602.7;
  Motor_1_Data.kvalhold=3.06;
  Motor_1_Data.kvalrun=3.06;
  Motor_1_Data.kvalacc=3.06;
  Motor_1_Data.kvaldec=3.06;
  Motor_1_Data.intspeed=61.52;
  Motor_1_Data.stslp=392.1569e-6;
  Motor_1_Data.fnslpacc=643.1372e-6;
  Motor_1_Data.fnslpdec=643.1372e-6;
  Motor_1_Data.kterm=0;
  Motor_1_Data.ocdth=1*1500*1.00;
  Motor_1_Data.stallth=800*1.00;
  Motor_1_Data.step_sel=MICROSTEP_1_16;
  Motor_1_Data.alarmen=0xFF;
  Motor_1_Data.config=0x2E88;

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



  Motor_1_Handler.DaisyChainPosition=0;
  Motor_1_Handler.Command=&L6470Command;
  L6470_Config(&Motor_1_Handler,&Motor_1_Data);


  Motor_2_Handler.DaisyChainPosition=1;
  Motor_2_Handler.Command=&L6470Command;
  L6470_Config(&Motor_2_Handler,&Motor_1_Data);

  Motor_3_Handler.DaisyChainPosition=2;
  Motor_3_Handler.Command=&L6470Command;
  L6470_Config(&Motor_3_Handler,&Motor_1_Data);

  Motor_4_Handler.DaisyChainPosition=3;
  Motor_4_Handler.Command=&L6470Command;
  L6470_Config(&Motor_4_Handler,&Motor_1_Data);


  Ring_Buffer_Init(&huart2);


  //L6470_Move(0,L6470_DIR_FWD_ID,1000);
  //L6470_Move(1,L6470_DIR_FWD_ID,1000);

  //L6470_Move(2,L6470_DIR_FWD_ID,1000);
  //L6470_Move(3,L6470_DIR_FWD_ID,1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	if (HAL_GPIO_ReadPin(Push_Button_GPIO_Port, Push_Button_Pin)
		== GPIO_PIN_RESET)
	    {
	    //L6470_Move(0,L6470_DIR_FWD_ID,1000000);
	    //L6470_Move(1,L6470_DIR_FWD_ID,1000000);
	    //L6470_Move(2,L6470_DIR_FWD_ID,200*64*10);
	    //L6470_Move(3,L6470_DIR_FWD_ID,1000000);

	    Motor_2_Home();
	    }

	    UART_Loop();


	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {

    if (GPIO_Pin == L6470_Flag_INT_Pin)
	{
	uint16_t StatusRegister = L6470_GetStatus(2);
	if ((StatusRegister & STATUS_STEP_LOSS_A) == 0
		|| (StatusRegister & STATUS_STEP_LOSS_B) == 0)
	    {
	    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	    L6470_HardStop(2);
	    }

	}

    }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
