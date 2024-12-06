/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t Time_us;
uint8_t buffer[5];
uint8_t i; // bien i cho vong lap for
uint8_t checksum = 0;
uint8_t Temperature = 0;
uint8_t Humidity = 0;
uint8_t hienthi[30];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};	
	GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
}

// Ðang cau hình thanh ghi CNT cu 1ms dem len 1 don vi
// tao ham delay don vi us
void delay_us (uint16_t time)
{
	__HAL_TIM_SetCounter(&htim1, 0);
	HAL_TIM_Base_Start(&htim1);
	while(__HAL_TIM_GetCounter(&htim1) < time);
	HAL_TIM_Base_Stop(&htim1);

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Ban dau cau hinh chon DATA(PB9), muc high
		//MCU gui tin hieu bat dau cho DHT11
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
		delay_us(20);  // giu trang thai low it nhat 18us
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		/* do cáu hình ouput là open drain nên trang thái 1 cua PB9 la do Tro keo, ko phai do MCU
		nên la dung 1 timer de cho cho chan gpio keo len 1*/
	
		Set_Pin_Input(GPIOB, GPIO_PIN_9);
		/* Cho chan PB9 len cao */
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 10)  //cho toi da 10us
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))  // neu chan PB9 da len muc cao (==1) thoai vong lap
			{
				break;  //thoat khoi vong lap while hoac for
			}
		}
		/* kiem tra xem thoát vòng lap là do vuot 10us hay PB9 = 1
		Bang cach doc gia tri timer2*/
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 10);  // >10 dung luon, vao vong lap vo han
		HAL_TIM_Base_Stop(&htim2);
		 
		/* Doi dht11 phan hoi (o muc High) tu 20 - 40us*/
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 40)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 0)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 40 && Time_us < 5);  // tren thuc te muc HIGH chi duy tri dc trong 10us 
		HAL_TIM_Base_Stop(&htim2);
		
		/* Cho chan PB9 len HIGH, khoang 80us, lay gioi han 70 - 90us*/
			__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 90)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 90 && Time_us < 70);  
		HAL_TIM_Base_Stop(&htim2);
		/* Cho PB9 xuong LOW, 80us, cbj gui du lieu*/
				__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 95)  //cho khoang rong hon xiu
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 0)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 95 && Time_us < 75);  
		HAL_TIM_Base_Stop(&htim2);

		
		/*Nhan byte so 1*/
		for(i = 0 ; i < 8 ; i++)
		{
			// Cho PB9 len cao, khoang 50us
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 65) 
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 65 && Time_us < 45);  
		HAL_TIM_Base_Stop(&htim2);
			// cho PB9 xuong thap, kiem tra la bit 0 (26-28us) hay bit 1 (70us) -> lay khoang tu 10 -80
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 80)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 80 && Time_us < 10);
		buffer[0] <<= 1;
			if( Time_us < 45)  //bit 0;
			{
				buffer[0] &= ~1;
				//buffer[0] |= 0<<(7-i);   // Toan tu OR: |
			}
			else
			{
				buffer[0] |= 1;
				//buffer[0] |= 1<<(7-i); //kich thuoc 8bit, dich i bit, i=0 -> 10000000, i = 1 --> 10000000 OR 0100000 = 1100000
			}
		}
		/*Nhan byte so 2*/
			for(i = 0 ; i < 8 ; i++)
		{
			// Cho PB9 len cao, khoang 50us
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 65) 
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 65 && Time_us < 45);  
		HAL_TIM_Base_Stop(&htim2);
			// cho PB9 xuong thap, kiem tra la bit 0 (26-28us) hay bit 1 (70us) -> lay khoang tu 10 -80
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 80)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 80 && Time_us < 10);
		buffer[1] <<= 1;
			if( Time_us < 45)  //bit 0;
			{
				buffer[1] &= ~1;
				//buffer[1] |= 0<<(7-i);   // Toan tu OR: |
			}
			else
			{
				buffer[1] |= 1;
				//buffer[1] |= 1<<(7-i); //kich thuoc 8bit, dich i bit, i=0 -> 10000000, i = 1 --> 10000000 OR 0100000 = 1100000
			}
		}
		/*Nhan byte so 3*/
		for(i = 0 ; i < 8 ; i++)
		{
			// Cho PB9 len cao, khoang 50us
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 65) 
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 65 && Time_us < 45);  
		HAL_TIM_Base_Stop(&htim2);
			// cho PB9 xuong thap, kiem tra la bit 0 (26-28us) hay bit 1 (70us) -> lay khoang tu 10 -80
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 80)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 80 && Time_us < 10);
		buffer[2] <<= 1;
			if( Time_us < 45)  //bit 0;
			{
				buffer[2] &= ~1;
				//buffer[2] |= 0<<(7-i);   // Toan tu OR: |
			}
			else
			{
				buffer[2] |= 1;
				//buffer[2] |= 1<<(7-i); //kich thuoc 8bit, dich i bit, i=0 -> 10000000, i = 1 --> 10000000 OR 0100000 = 1100000
			}
		}
		/*Nhan byte so 4*/
			for(i = 0 ; i < 8 ; i++)
		{
			// Cho PB9 len cao, khoang 50us
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 65) 
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 65 && Time_us < 45);  
		HAL_TIM_Base_Stop(&htim2);
			// cho PB9 xuong thap, kiem tra la bit 0 (26-28us) hay bit 1 (70us) -> lay khoang tu 10 -80
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 80)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 80 && Time_us < 10);
		buffer[3] <<= 1;
			if( Time_us < 45)  //bit 0;
			{
				buffer[3] &= ~1;
				//buffer[3] |= 0<<(7-i);   // Toan tu OR: |
			}
			else
			{
				buffer[3] |= 1;
				//buffer[3] |= 1<<(7-i); //kich thuoc 8bit, dich i bit, i=0 -> 10000000, i = 1 --> 10000000 OR 0100000 = 1100000
			}
		}
		/*Nhan byte so 5*/
			for(i = 0 ; i < 8 ; i++)
		{
			// Cho PB9 len cao, khoang 50us
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 65) 
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 65 && Time_us < 45);  
		HAL_TIM_Base_Stop(&htim2);
			// cho PB9 xuong thap, kiem tra la bit 0 (26-28us) hay bit 1 (70us) -> lay khoang tu 10 -80
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GetCounter(&htim2) < 80)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == 1)  
			{
				break;  
			}
		}
		Time_us = __HAL_TIM_GetCounter(&htim2);
		while(Time_us >= 80 && Time_us < 10);
		buffer[4] <<= 1;
			if( Time_us < 45)  //bit 0;
			{
				buffer[4] &= ~1;
				//buffer[4] |= 0<<(7-i);   // Toan tu OR: |
			}
			else
			{
				buffer[4] |= 1;
				//buffer[4] |= 1<<(7-i); //kich thuoc 8bit, dich i bit, i=0 -> 10000000, i = 1 --> 10000000 OR 0100000 = 1100000
			}
		}
		
		//Dung led PC13 de kiem tra Tin hieu start da dung, khong bi vao vong lap while
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		HAL_Delay(500);
		
		checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3];
		while(checksum != buffer[4]);
		
		/*gian gia tr Nhiet do va do am*/
		Temperature = buffer[2];
		Humidity = buffer [0];
		
		sprintf((char *)hienthi, "nhiet do %d", buffer[2]);
		HAL_UART_Transmit(&huart1,hienthi, strlen((char *)hienthi), 300);
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
