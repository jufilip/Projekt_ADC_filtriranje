/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

/* USER CODE BEGIN PV */
__IO uint16_t buffer[BUFFER_SIZE];			//Tu cu spremati digitalizirani signal iz ADC-a
__IO uint16_t array[BUFFER_SIZE];			//Tu cu spremati digitalizirani signal iz buffera spreman za filtriranje
__IO uint16_t filteredArray[BUFFER_SIZE];	//Tu cu spremati isfiltrirani signal
__IO uint16_t firCoef_HP[BUFFER_SIZE + 2] = {0.0008, -0.0005, 0, 0.0006, -0.0012, 0.0015, -0.0011, 0, 0.0017, -0.0033, 0.0039, -0.0029, 0, 0.0040,		//Tu cu spremiti izracunate koeficijente visokopropusnog fir filtra iz matlaba 	fir1(63, 0.8, 'high')
											-0.0076, 0.0088, -0.0063, 0, 0.0084, -0.0157, 0.0181, -0.0129, 0, 0.0173, -0.0327,  0.0387,-0.0288, 0,
											 0.0451, -0.0989, 0.1500, -0.1867, 0.2000, -0.1867, 0.1500, -0.0989, 0.0451, 0, -0.0288, 0.0387, -0.0327, 0.0173,
											 0, -0.0129, 0.0181, -0.0157, 0.0084, 0, -0.0063, 0.0088, -0.0076, 0.0040, 0, -0.0029, 0.0039, -0.0033,
											 0.0017, 0, -0.0011, 0.0015, -0.0012, 0.0006, 0, -0.0005, 0.0008};
}
__IO uint16_t firCoef_LP[BUFFER_SIZE] = {0.0007, 0.0003, -0.0003, -0.0009, -0.0013, -0.0013, -0.0006, 0.0008, 0.0024, 0.0036, 0.0035, 0.0016, -0.0019, -0.0058,		//Tu cu spremiti izracunate koeficijente niskopropusnog fir filtra iz matlaba fir1(63, 0.2)
											 -0.0084, -0.0079, -0.0035, 0.0041, 0.0123, 0.0175, 0.0163, 0.0072, -0.0084, -0.0256, -0.0373, -0.0359, -0.0167, 0.0209,
											  0.0716, 0.1256, 0.1709, 0.1967, 0.1967, 0.1709, 0.1256, 0.0716, 0.0209, -0.0167, -0.0359, -0.0373, -0.0256, -0.0084,
											  0.0072, 0.0163, 0.0175, 0.0123, 0.0041, -0.0035, -0.0079, -0.0084, -0.0058, -0.0019, 0.0016, 0.0035, 0.0036, 0.0024,
											  0.0008, -0.0006, -0.0013, -0.0013, -0.0009, -0.0003, 0.0003, 0.0007};

_Bool filterType = 0;						//Varijabla za odabir vrste filtra (1->HP, 0->LP), Stavio sam zasad u 0 da se koristi niski filtar za testiranje
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
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
  firCoefInit();
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
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&buffer, BUFFER_SIZE) != HAL_OK)
    {
      /* Start Conversation Error */
      Error_Handler();
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	for(int i = 0; i < BUFFER_SIZE; ++i)
		array[i] = buffer[i];

	float sum = 0;

	if(filterType = 1) {								//filterType = 1 -> HP
		for(int n = 0; n < BUFFER_SIZE; ++n) {			//L = BUFFER_SIZE
			for(int k = 0; k < BUFFER_SIZE; ++k) {		//N = L - 1 = BUFFER_SIZE - 1
				if(n - k >= 0)
					sum += firCoef_HP[k] * array[n-k];
				else
					sum = 0;
			}
			filteredArray[n] = sum;
			sum = 0;
		}
	} else {											//filterType = 0 -> LP
		for(int n = 0; n < BUFFER_SIZE; ++n) {
			for(int k = 0; k < BUFFER_SIZE; ++k) {
				if(n - k >= 0)
					sum += firCoef_LP[k] * array[n-k];
				else
					sum = 0;
			}
			filteredArray[n] = sum;
			sum = 0;
		}
	}
}

void firCoefInit(void) {
	//PUNJENJE HP FILTRA S KOEFICIJENTIMA:

	//PUNJENJE LP FILTRA S KOEFICIJENTIMA:
	//firCoef_LP =



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
