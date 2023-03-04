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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"
#include "string.h"
#include "modbus.h"
#include "Lcd.h"
#include "Lora.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_RTU_MIN_LENGTH 7
#define MODBUS_RTU_MAX_LENGTH 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueueData */
osMessageQueueId_t myQueueDataHandle;
const osMessageQueueAttr_t myQueueData_attributes = {
  .name = "myQueueData"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* Definitions for myCountingSem01 */
osSemaphoreId_t myCountingSem01Handle;
const osSemaphoreAttr_t myCountingSem01_attributes = {
  .name = "myCountingSem01"
};
/* USER CODE BEGIN PV */
static uint16_t Data_16[2];
static uint32_t Data_32;
static uint8_t buffer[9];
static uint8_t rxData,Error_T, Error_H,Error_Device;
static uint8_t rx_data[MODBUS_RTU_MAX_LENGTH];
static int rx_index = 0;
static uint16_t rx_length = 0;
static uint16_t Error_Stat, Error_Cod;
static float Temp, RH;
static int i = 0, a = 0;
static char LCD_Send[30];
static CLCD_Name LCD1;
static LoRa myLoRa;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void LoRa_Setup(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		static BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(myQueueDataHandle,&buffer,NULL);
		xSemaphoreGiveFromISR(myCountingSem01Handle,&xHigherPriorityTaskWoken);
		HAL_UART_Receive_IT(&huart2, buffer, 1);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B_Temp_Pin)
	{
		 static uint32_t last_interrupt_time = 0;
		 uint32_t current_time = HAL_GetTick();
		 if ((current_time - last_interrupt_time) > 200)
		 {
			 a = 1;
			 Send_Error_Status();
			 HAL_GPIO_WritePin(GPIOB, Led_Temp_Pin, GPIO_PIN_SET);
//			 HAL_GPIO_TogglePin(GPIOB, Led_Temp_Pin);
		 }
		 last_interrupt_time = current_time;
	}
	else if (GPIO_Pin == B_Water_Pin)
	{
		static uint32_t last_interrupt_time = 0;
		uint32_t current_time = HAL_GetTick();
		if ((current_time - last_interrupt_time) > 200)
		{
			a = 1;
			Send_Error_Status();
			HAL_GPIO_WritePin(GPIOB, Led_Water_Pin, GPIO_PIN_SET);
//			HAL_GPIO_TogglePin(GPIOB, Led_Water_Pin);
		}
		last_interrupt_time = current_time;
	}
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  CLCD_4BIT_Init(&LCD1, 16, 2, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin,
    									D4_GPIO_Port, D4_Pin, D5_GPIO_Port, D5_Pin,
    									D6_GPIO_Port, D6_Pin, D7_GPIO_Port, D7_Pin);
  LoRa_Setup();
  LoRa_init(&myLoRa);
  LoRa_startReceiving(&myLoRa);
  HAL_UART_Receive_IT(&huart2, buffer, 1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* creation of myCountingSem01 */
  myCountingSem01Handle = osSemaphoreNew(5, 5, &myCountingSem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueueData */
  myQueueDataHandle = osMessageQueueNew (16, sizeof(uint16_t), &myQueueData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Blink_Pin|D7_Pin|D6_Pin|Led_Water_Pin
                          |Led_Temp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D5_Pin|D4_Pin|EN_Pin|RW_Pin
                          |RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B_Temp_Pin B_Water_Pin */
  GPIO_InitStruct.Pin = B_Temp_Pin|B_Water_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin RST_Pin Blink_Pin D7_Pin
                           D6_Pin Led_Water_Pin Led_Temp_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin|Blink_Pin|D7_Pin
                          |D6_Pin|Led_Water_Pin|Led_Temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D5_Pin D4_Pin EN_Pin RW_Pin
                           RS_Pin */
  GPIO_InitStruct.Pin = D5_Pin|D4_Pin|EN_Pin|RW_Pin
                          |RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void Process_Modbus(void)
{
	static uint8_t LoRa_TxBuffer[10];
	switch(i)
	{
	case 0:
		Error_Stat = Receive_Error_Status(Data_16[0]);
		if (Error_Stat == Have_Errors)
		{
			osDelay(100);
			Send_Error_Code();
			i = 1;
		}
		else // Read Temp
		{
//			CLCD_SetCursor(&LCD1, 0, 0);
//			CLCD_WriteString(&LCD1,"No Errors");
			osDelay(100);
			memset(rx_data, 0, sizeof(rx_data));
			Read_Temp();
			i = 2;
		}
		break;

	// Check Error code
	case 1:
		Error_Cod = Receive_Error_Code(Data_16[0]);
		// Display error
		if (Error_Cod == Temp_Measur_Err)
		{
			CLCD_SetCursor(&LCD1, 0, 0);
			CLCD_WriteString(&LCD1,"Temp Measure Err");
			osDelay(2000);
			LoRa_TxBuffer[0] = 0x01;
			LoRa_TxBuffer[1] = 0x01; // Error
			LoRa_TxBuffer[2] = 0x01; // Temp Measure Err
			LoRa_transmit(&myLoRa, LoRa_TxBuffer, 3, 1000);
			memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
			memset(rx_data, 0, sizeof(rx_data));
			Send_Error_Temp();
			i = 3;
			}
		else if (Error_Cod == Hum_Measur_Err)
		{
			CLCD_SetCursor(&LCD1, 0, 0);
			CLCD_WriteString(&LCD1,"Humi Measure Err");
			osDelay(2000);
			LoRa_TxBuffer[0] = 0x01;
			LoRa_TxBuffer[1] = 0x01; // Error
			LoRa_TxBuffer[2] = 0x02; // Humi Measure Err
			LoRa_transmit(&myLoRa, LoRa_TxBuffer, 3, 1000);
			memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
			memset(rx_data, 0, sizeof(rx_data));
			Send_Error_RH();
			i = 4;
		}
		else if (Error_Cod == Humi_Sen_Fail)
		{
			CLCD_SetCursor(&LCD1, 0, 0);
			CLCD_WriteString(&LCD1,"Humi Sensor Fail ");
			osDelay(2000);
			LoRa_TxBuffer[0] = 0x01;
			LoRa_TxBuffer[1] = 0x01; // Error
			LoRa_TxBuffer[2] = 0x03; // Humi Sensor Fail
			LoRa_transmit(&myLoRa, LoRa_TxBuffer, 3, 1000);
			memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
			memset(rx_data, 0, sizeof(rx_data));
			Send_Error_Device();
			i = 5;
		}
		break;

	// Read Temp
	case 2:
		LoRa_TxBuffer[0] = 0x01;
		LoRa_TxBuffer[1] = 0x02; // Temp
		LoRa_TxBuffer[2] = rx_data[3];
		LoRa_TxBuffer[3] = rx_data[4];
		LoRa_TxBuffer[4] = rx_data[5];
		LoRa_TxBuffer[5] = rx_data[6];
//		LoRa_transmit(&myLoRa, LoRa_TxBuffer, 6, 1000);
//		memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
		Temp = unpack754_32(Data_32);
		sprintf(LCD_Send,"Temp: %.2f oC",Temp );
		CLCD_SetCursor(&LCD1, 0, 0);
		CLCD_WriteString(&LCD1, LCD_Send);
		osDelay(1000);
		memset(rx_data, 0, sizeof(rx_data));
		Temp = 0;
		Read_Water();
		i = 6;
		break;

	// Read Error Temperature measurement
	case 3:
		Error_T = Receive_Error_Temp(Data_16[0]);
		// Display error
		LoRa_TxBuffer[0] = 0x01;
		LoRa_TxBuffer[1] = 0x01; // Error
		LoRa_TxBuffer[2] = 0x01; // Temp Measure Err
		LoRa_TxBuffer[3] = Error_T;
		LoRa_transmit(&myLoRa, LoRa_TxBuffer, 4, 1000);
		memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
		memset(rx_data, 0, sizeof(rx_data));
		break;

	// Read Error RH measurement
	case 4:
		Error_H = Receive_Error_RH(Data_16[0]);
		// Display error
		LoRa_TxBuffer[0] = 0x01;
		LoRa_TxBuffer[1] = 0x01; // Error
		LoRa_TxBuffer[2] = 0x02; // RH Measure Err
		LoRa_TxBuffer[3] = Error_H;
		LoRa_transmit(&myLoRa, LoRa_TxBuffer, 4, 1000);
		memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
		memset(rx_data, 0, sizeof(rx_data));
		break;

	// Read error device
	case 5:
		Error_Device = Receive_Error_Device(Data_16[0]);
		// Display error
		LoRa_TxBuffer[0] = 0x01;
		LoRa_TxBuffer[1] = 0x01; // Error
		LoRa_TxBuffer[2] = 0x02; // RH Measure Err
		LoRa_TxBuffer[3] = Error_Device;
		LoRa_transmit(&myLoRa, LoRa_TxBuffer, 4, 1000);
		memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
		memset(rx_data, 0, sizeof(rx_data));
		break;

	// Read RH
	case 6:
//		LoRa_TxBuffer[0] = 0x02;
//		LoRa_TxBuffer[1] = 0x03; // Water
		LoRa_TxBuffer[6] = rx_data[3];
		LoRa_TxBuffer[7] = rx_data[4];
		LoRa_TxBuffer[8] = rx_data[5];
		LoRa_TxBuffer[9] = rx_data[6];
		LoRa_transmit(&myLoRa, LoRa_TxBuffer, 10, 1000);
		memset(LoRa_TxBuffer, 0, sizeof(LoRa_TxBuffer));
		RH =  unpack754_32(Data_32);
		sprintf(LCD_Send,"Water: %.2f ppm",RH );
		CLCD_SetCursor(&LCD1, 0, 1);
		CLCD_WriteString(&LCD1, LCD_Send);
		HAL_Delay(5000);
//		CLCD_Clear(&LCD1);
		HAL_GPIO_WritePin(GPIOB, Led_Water_Pin|Led_Temp_Pin, GPIO_PIN_RESET);
		RH = 0;
		memset(rx_data, 0, sizeof(rx_data));
		i = 0;
	}
}

void LoRa_Setup(void)
{
	  myLoRa.CS_port         = NSS_GPIO_Port;
	  myLoRa.CS_pin          = NSS_Pin;
	  myLoRa.reset_port      = RST_GPIO_Port;
	  myLoRa.reset_pin       = RST_Pin;
	  myLoRa.DIO0_port       = DIO0_GPIO_Port;
	  myLoRa.DIO0_pin        = DIO0_Pin;
	  myLoRa.hSPIx           = &hspi1;
	  myLoRa.frequency             = 433       ;
	  myLoRa.spredingFactor        = SF_7      ;
	  myLoRa.bandWidth			   = BW_125KHz ;
	  myLoRa.crcRate               = CR_4_5    ;
	  myLoRa.power				   = POWER_20db;
	  myLoRa.overCurrentProtection = 100       ;
	  myLoRa.preamble			   = 8         ;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	while(xSemaphoreTake(myCountingSem01Handle,portMAX_DELAY))
		{
			if (xQueueReceive(myQueueDataHandle, &rxData, portMAX_DELAY))
			{
				rx_data[rx_index++] = rxData;
				rx_length++;
				if (rx_length >= 7 && checkcrc16(rx_data, rx_index) == 1)
				{
					Data_16[0] = rx_data[3]<<8 | rx_data[4];
					Data_16[1] = rx_data[5]<<8 | rx_data[6];
					Data_32 = Data_16[1]<<16 | Data_16[0];
					Process_Modbus();
					rx_index = 0;
					rx_length = 0;
				}
			}
		}
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {
    Send_Error_Status();
  }

  /* USER CODE END Callback 1 */
}

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
