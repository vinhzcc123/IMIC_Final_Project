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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "custom.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void flashErase(uint32_t address, uint8_t numOfSector) __attribute__((section(".RamFunc")));
void flashWrite(uint32_t address, uint16_t writeData) __attribute__((section(".RamFunc")));
int waitOperation() __attribute__((section(".RamFunc")));
void update_firmware() __attribute__((section(".RamFunc")));
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
__attribute__((section(".firtlinevector"))) char vector_tablee [1024];
extern char buffer[4096];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  custom_clock_init();
  custom_UART_init();
  custom_DMA();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
   memcpy(vector_tablee, 0x08000000, 512);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  //flashErase(0x08000000, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  UART_write('x');
	  custom_delay(1000);
	  if(find_OK())
	  {
		  update_firmware();
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void flashErase(uint32_t address, uint8_t numOfSector)
{
	uint32_t* FLASH_SR = (uint32_t*)0x4002200C;
	uint32_t* FLASH_CR = (uint32_t*)0x40022010;
	uint32_t* FLASH_AR = (uint32_t*)0x40022014;
	HAL_FLASH_Unlock();
	if(((*FLASH_CR >> 1) & 1 )!= 1)
	{
		*FLASH_CR |= 1<<1;		//Page Erase chosen
	}
	if((*FLASH_CR & 1) == 1)
	{
		*FLASH_CR &= ~(1UL);
	}
	for(int i = 0; i < numOfSector; i++)
	{
		*FLASH_CR &= ~1UL;
		while(((*FLASH_SR) & 1) == 1);
		*FLASH_AR = address + 0x400 * i;
		*FLASH_CR |=  1<<6;		//Start

		while(((*FLASH_SR >> 5) & 1) != 1);
		*FLASH_SR |= 1<<5;
	}

	*FLASH_CR &= ~(1u<<1);
}

void flashWrite(uint32_t address, uint16_t writeData)
{
	uint32_t* FLASH_CR = (uint32_t*)0x40022010;
	volatile uint16_t* pData;
	*FLASH_CR |= 1;
	pData = address;
	waitOperation();

	*pData = writeData;
	waitOperation();
	*FLASH_CR &= ~1u;
}

int waitOperation()
{
	uint32_t* FLASH_SR = (uint32_t*)0x4002200C;
	uint32_t* FLASH_WRPR = (uint32_t*)0x4002201C;
	while(*FLASH_SR & 1);

	if((*FLASH_SR<<5) & 1)
	{
		*FLASH_SR |= 1<<5;
	}

	if(((*FLASH_WRPR>>1)&1)  ||((*FLASH_WRPR)&1) ||((*FLASH_SR>>2)&1))
	{
		while(1);
	}

	return HAL_OK;
}

void update_firmware()
{
//	while(strstr(buffer, "OK") == 0);
	__HAL_RCC_FLITF_CLK_ENABLE();
	HAL_FLASH_Unlock();
	flashErase(0x08000000, 4);
	int size = sizeof(buffer);
	for(int i = 0; i < size; i+=2)
	{
		flashWrite(0x08000000 + i, *(uint16_t*)(buffer+i));
	}
	uint32_t* AIRCR = (uint32_t*)0xE000ED0C;
	*AIRCR = (0x5FA<<16) | (1<<2);
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
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
