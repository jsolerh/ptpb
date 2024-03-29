
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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "flash_mem.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

BNO_measure actual_measure;
BNO_measure flash_measure;
int32_t n_measures;
int32_t pointer_flash;
char buffer[100];
int len;
extern volatile int8_t onoffB1;
extern volatile int8_t onoffB2;
int8_t estado;
int i;
uint32_t last_B1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int8_t ISR_B1(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	BNO_Init();
//	Flash_Chip_Erase();
	Flash_Read_Id();
//	Flash_Read_Addr(1);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
//	Flash_Chip_Erase();
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	n_measures = 0;
	pointer_flash = 0;
	len=strlen(buffer);
	estado=0;
	onoffB1=0;
	onoffB2=0;
	last_B1=0;
//	HAL_UART_Transmit(&huart1, buffer, len, 1000);
//	sprintf(buffer, "LinearX LinearY LinearZ GravityX GravityY GravityZ EulerX EulerY EulerZ\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
estado = ISR_B1();
if (onoffB2) estado = 2;
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		switch (estado){

		case 0:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		break;


		case 1 :
				if (pointer_flash <= 6480000 ) {
					BNO_Mide(&actual_measure);
					Flash_Write_Measure(pointer_flash, &actual_measure);
					n_measures++;
					pointer_flash = n_measures * 18;
				}
			break;

		case 2:

			if (n_measures != 0) {
				len=strlen(buffer);
				HAL_UART_Transmit(&huart1, buffer, len, 1000);
				sprintf(buffer, "%i	\r\n", n_measures);
			}

			for (i=0; i <= n_measures; i++){
				if (n_measures != 0) {
					Flash_Read_Measure(i*18, &flash_measure);
					len=strlen(buffer);
					HAL_UART_Transmit(&huart1, buffer, len, 1000);
					sprintf(buffer, "%i	%i	%i	%i	%i	%i	%i	%i	%i\r\n", flash_measure.BNO_LinearX, flash_measure.BNO_LinearY, flash_measure.BNO_LinearZ, flash_measure.BNO_GyroscopeX, flash_measure.BNO_GyroscopeY, flash_measure.BNO_GyroscopeZ, flash_measure.BNO_EulerX, flash_measure.BNO_EulerY, flash_measure.BNO_EulerZ);
					if (i == n_measures){
						Flash_Chip_Erase();
						pointer_flash = 0;
						n_measures=0;
						estado = 0;
						onoffB2=0;
					}
				}
			}
//			i = 0;
//			Flash_Chip_Erase();
			pointer_flash = 0;
			n_measures=0;
			estado = 0;
			onoffB2=0;
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//			onoff = 0;
			break;

//		default:
//			HAL_UART_Transmit(&huart1, buffer, len, 1000);
//			sprintf(buffer, "Error");


		}

//		Flash_Read_Measure(pointer_flash, &flash_measure);
//		BNO_Mide(&actual_measure);
//		Flash_Write_Measure(pointer_flash, &actual_measure);
//		Flash_Read_Measure(pointer_flash, &flash_measure);
//		n_measures++;
//		pointer_flash = n_measures * 18;
//		len=strlen(buffer);
//		HAL_UART_Transmit(&huart1, buffer, len, 1000);
//		sprintf(buffer, "%i	%i	%i	%i	%i	%i	%i	%i	%i\r\n", flash_measure.BNO_LinearX, flash_measure.BNO_LinearY, flash_measure.BNO_LinearZ, flash_measure.BNO_GravityX, flash_measure.BNO_GravityY, flash_measure.BNO_GravityZ, flash_measure.BNO_EulerX, flash_measure.BNO_EulerY, flash_measure.BNO_EulerZ);
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
int8_t ISR_B1(void){
	if(onoffB1) {
	if ((HAL_GetTick() - last_B1) > 1000) {
		estado = !estado;}
	onoffB1=0;
	last_B1 = HAL_GetTick();
	}
	return estado;
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
