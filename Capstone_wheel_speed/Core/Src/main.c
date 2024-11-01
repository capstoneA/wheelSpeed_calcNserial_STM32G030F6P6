/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned int led_check = 0;
unsigned char speed = 0;

unsigned int queue[20] = {0};
int queue_front = 0;
int queue_rear  = 1;

int isQueueFull(){
	if(queue_front == queue_rear)return 1;
	return 0;
}

void dequeue(){
	queue_front++;
	if(queue_front==20)queue_front=0;
	return;
}

void enqueue(unsigned int v){
	if(isQueueFull()){
		dequeue();
	}
	queue[queue_rear-1] = v;
	queue_rear++;
	if(queue_rear == 20)queue_rear = 0;
	return;
}

void timer1setting(){
	 *(volatile unsigned int*)(0x40021040) |= 0x01<<11;	//RCC

	 *(volatile unsigned int*)(0x40012C0C) |= 0x01<<0;	//DIER->UIE
	 *(volatile unsigned int*)(0x40012C10) &= ~(0x01<<0);	//SR - UIF

//	 *(volatile unsigned int*)(0x40012C24);	//CNT
	 *(volatile unsigned int*)(0x40012C28) = 63;	//Prescaler
//	 *(volatile unsigned int*)(0x40012C2C);	//ARR

	 *(volatile unsigned int*)(0x40012C00) |= 0x01<<0;	//counter enable

	 *(volatile unsigned int*)(0xE000E100) |= 0x01<<13;	//NVIC enable
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(){
	if(*(volatile unsigned int*)(0x40012C10) & (0x01<<0)){
		*(volatile unsigned int*)(0x40012C10) &= ~(0x01<<0);	//SR - UIF

//		*(volatile unsigned int*)(0x40013828) = 97;

		speed = 0;
		*(volatile unsigned int*)(0x40013828) = speed;

		led_check^=1;
		if(led_check){
		  *(volatile unsigned int*)(0x50000018) = 0x01<<(16+4);
		}
		if(!led_check){
		  *(volatile unsigned int*)(0x50000018) = 0x01<<(4);
		}
	}
}

void PA8inputSetting(){
	*(volatile unsigned int*)(0x40021034) |= 0x01<<0;			//clock enable
	*(volatile unsigned int*)(0x50000000) &= ~(0x03<<(8*2));	//input
    *(volatile unsigned int*)(0x5000000C) &= ~(0x03<<(8*2));	//pupd - not
    *(volatile unsigned int*)(0x50000008) |= 0x03<<(8*2);		//speed very fast
}

void EXTI8setting(){
	*(volatile unsigned int*)(0x40021800) |= 0x01<<8;	//rising edge trigger
	*(volatile unsigned int*)(0x40021868) &= ~(0xF<<0);	//use PA8

	*(volatile unsigned int*)(0x40021880) |= 0x01<<8;	//IMR

	*(volatile unsigned int*)(0xE000E100) |= 0x01<<7;	//EXTI set enable reg
}

void EXTI4_15_IRQHandler(){
	if(*(volatile unsigned int*)(0x4002180C) & (0x01<<8)){

//		enqueue(*(volatile unsigned int*)(0x40012C24));
//		*(volatile unsigned int*)(0x40012C24) = 0x00;

		*(volatile unsigned int*)(0x4002180C) |= 0x01<<8;

//		*(volatile unsigned int*)(0x40013828) = 97;

		unsigned int cnt = *(volatile unsigned int*)(0x40012C24) & 0xFFFF;
		*(volatile unsigned int*)(0x40012C24) = 0x00;
		speed = (unsigned char)(190625/cnt);
		if(speed){
				//speed update when last speed is not 0
			*(volatile unsigned int*)(0x40013828) = speed;
		}

		led_check^=1;
		if(led_check){
		  *(volatile unsigned int*)(0x50000018) = 0x01<<(16+4);
		}
		if(!led_check){
		  *(volatile unsigned int*)(0x50000018) = 0x01<<(4);
		}
	}
}

void UART1Setting(){
	*(volatile unsigned int*)(0x40021040) |= 0x01<<14;	//RCC uart1 enable
	*(volatile unsigned int*)(0x40021034) |= 0x01<<1;	//RCC GPIOB clock enable

	*(volatile unsigned int*)(0x50000400) |= 0x02<<(6*2);	//MODER
	*(volatile unsigned int*)(0x50000400) &= ~(0x01<<(6*2));	//MODER
	*(volatile unsigned int*)(0x50000400) |= 0x02<<(7*2);	//MODER
	*(volatile unsigned int*)(0x50000400) &= ~(0x01<<(7*2));	//MODER

	*(volatile unsigned int*)(0x50000420) &= ~(0x0F<<(7*4));	//AFRL
	*(volatile unsigned int*)(0x50000420) &= ~(0x0F<<(6*4));	//AFRL

	*(volatile unsigned int*)(0x4001380C) = 1667;	//baud rate : 16000000/9600
	*(volatile unsigned int*)(0x40013800) |= 0x01<<0;	//UART Enable
	*(volatile unsigned int*)(0x40013800) |= 0x01<<3;	//UART Transmitter Enable
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

  //190625
  /* USER CODE BEGIN 2 */

  *(volatile unsigned int*)(0x40021034) |= 0x01<<0;			//PA Clock Enable
  *(volatile unsigned int*)(0x50000000) |= (0x01)<<(4*2);	//MODER
  *(volatile unsigned int*)(0x50000000) &= ~(0x02)<<(4*2);	//MODER
  *(volatile unsigned int*)(0x50000008) |= (0x03)<<(4*2);	//speed - very fast
  *(volatile unsigned int*)(0x5000000C) &= ~(0x03)<<(4*2);	//pupd - not

  /* USER CODE END 2 */

  PA8inputSetting();
  EXTI8setting();
  timer1setting();
  UART1Setting();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ;
    /* USER CODE END WHILE */
//	  *(volatile unsigned int*)(0x50000018) = 0x01<<(16+4);
//	  for(volatile unsigned int i=0;i<100000;i++);
//	  *(volatile unsigned int*)(0x50000018) = 0x01<<(4);
//	  for(volatile unsigned int i=0;i<100000;i++);

//	  if(*(volatile unsigned int*)(0x50000010) & 0x0100){
//		 *(volatile unsigned int*)(0x50000018) = 0x01<<(4);
//	  }
//	  else{
//		  *(volatile unsigned int*)(0x50000018) = 0x01<<(16+4);
//	  }

//	  int index = queue_front;
//	  unsigned int timing = queue[index];
//	  unsigned int mps = n/timing;

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim1.Init.Prescaler = 0;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PB7 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB7);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB6);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
