/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "snow_tiger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWO_define (0u)
#define USART_defined (1u)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
extern void ILI9341_myInit(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */
void Task_action(char message);
void Perfomance_Test(void);
void Counting_Multiple_Segments_Test(void);
void Counting_Single_Segments_Test(void);
void Alignment_Test(void);
void Lines_Example_Test(void);
void Hollow_Circles_Test(void);
void Filled_Circles_Test(void);
void Hollow_Rectangles_Test(void);
void Filled_Rectangles_Test(void);
void Individual_Pixel_Test(void);
void Individual2_Pixel_Test(void);
void Colour_Test(void);
void Image_Snow_Tiger_Test(void);
void TouchScreen_Test(void);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  ILI9341_myInit();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(StartTask01, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  htim1.Init.Prescaler = 10000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RESET_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DCX_Pin|T_CS_Pin|T_CLK_Pin|T_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DCX_Pin T_CS_Pin T_CLK_Pin T_MOSI_Pin */
  GPIO_InitStruct.Pin = LCD_DCX_Pin|T_CS_Pin|T_CLK_Pin|T_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : T_IRQ_Pin T_MISO_Pin */
  GPIO_InitStruct.Pin = T_IRQ_Pin|T_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Task_action(char message)
{
#if (SWO_define == (1u))
	ITM_SendChar(message);
	ITM_SendChar('\n');
#elif (USART_defined == (1u))
	uint8_t data[] = {message, '\n', '\r'};
	HAL_UART_Transmit(&huart2, data, 3, 2);
#else
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Task_action('!');
}
void Perfomance_Test(void)
{
	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("FPS TEST, 40 loop 2 screens", 10, 10, BLACK, 1, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(WHITE);

	uint32_t Timer_Counter = 0;
	for(uint32_t j = 0; j < 2; j++)
	{
		HAL_TIM_Base_Start(&htim1);
		for(uint16_t i = 0; i < 10; i++)
		{
			ILI9341_Fill_Screen(WHITE);
			ILI9341_Fill_Screen(BLACK);
		}

		//20.000 per second!
		HAL_TIM_Base_Stop(&htim1);
		Timer_Counter += __HAL_TIM_GET_COUNTER(&htim1);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
	}
	Timer_Counter /= 2;

	char counter_buff[30];
	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	sprintf(counter_buff, "Timer counter value: %d", Timer_Counter*2);
	ILI9341_Draw_Text(counter_buff, 10, 10, BLACK, 1, WHITE);

	double seconds_passed = 2*((float)Timer_Counter / 20000);
	sprintf(counter_buff, "Time: %.3f Sec", seconds_passed);
	ILI9341_Draw_Text(counter_buff, 10, 30, BLACK, 2, WHITE);

	double timer_float = 20/(((float)Timer_Counter)/20000);	//Frames per sec

	sprintf(counter_buff, "FPS:  %.2f", timer_float);
	ILI9341_Draw_Text(counter_buff, 10, 50, BLACK, 2, WHITE);
	double MB_PS = timer_float*240*320*2/1000000;
	sprintf(counter_buff, "MB/S: %.2f", MB_PS);
	ILI9341_Draw_Text(counter_buff, 10, 70, BLACK, 2, WHITE);
	double SPI_utilized_percentage = (MB_PS/(6.25 ))*100;		//50mbits / 8 bits
	sprintf(counter_buff, "SPI Utilized: %.2f", SPI_utilized_percentage);
	ILI9341_Draw_Text(counter_buff, 10, 90, BLACK, 2, WHITE);
	HAL_Delay(10000);

}
void Counting_Multiple_Segments_Test(void)
{
	char Temp_Buffer_text[40];

	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("Counting multiple segments at once", 10, 10, BLACK, 1, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(WHITE);


	for(uint16_t i = 0; i <= 10; i++)
	{
	sprintf(Temp_Buffer_text, "Counting: %d", i);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 30, BLUE, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 50, RED, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 70, GREEN, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 90, BLACK, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 110, BLUE, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 130, RED, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, GREEN, 2, WHITE);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 170, WHITE, 2, BLACK);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 190, BLUE, 2, BLACK);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 210, RED, 2, BLACK);
	}

	HAL_Delay(1000);
}
void Counting_Single_Segments_Test(void)
{
	char Temp_Buffer_text[40];

	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("Counting single segment", 10, 10, BLACK, 1, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(WHITE);

	for(uint16_t i = 0; i <= 100; i++)
	{
	sprintf(Temp_Buffer_text, "Counting: %d", i);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 3, WHITE);
	}

	HAL_Delay(1000);
}
void Alignment_Test(void)
{
	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("Rectangle alignment check", 10, 10, BLACK, 1, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(WHITE);

	ILI9341_Draw_Hollow_Rectangle_Coord(50, 50, 100, 100, BLACK);
	ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 50, 50, BLACK);
	ILI9341_Draw_Hollow_Rectangle_Coord(10, 10, 19, 19, BLACK);
	HAL_Delay(1000);
}
void Lines_Example_Test(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("Horizontal and Vertical lines", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//	for(uint32_t i = 0; i < 30000; i++)
//	{
//		uint32_t random_num = 0;
//		uint16_t xr = 0;
//		uint16_t yr = 0;
//		uint16_t radiusr = 0;
//		uint16_t colourr = 0;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		xr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		yr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		radiusr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		colourr = random_num;
//
//		xr &= 0x01FF;
//		yr &= 0x01FF;
//		radiusr &= 0x001F;
//		//ili9341_drawpixel(xr, yr, WHITE);
//		ILI9341_Draw_Horizontal_Line(xr, yr, radiusr, colourr);
//		ILI9341_Draw_Vertical_Line(xr, yr, radiusr, colourr);
//	}
//
//	HAL_Delay(1000);
}
void Hollow_Circles_Test(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("Circles", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//
//	for(uint32_t i = 0; i < 3000; i++)
//	{
//		uint32_t random_num = 0;
//		uint16_t xr = 0;
//		uint16_t yr = 0;
//		uint16_t radiusr = 0;
//		uint16_t colourr = 0;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		xr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		yr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		radiusr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		colourr = random_num;
//
//		xr &= 0x01FF;
//		yr &= 0x01FF;
//		radiusr &= 0x001F;
//		//ili9341_drawpixel(xr, yr, WHITE);
//		ILI9341_Draw_Hollow_Circle(xr, yr, radiusr*2, colourr);
//	}
//	HAL_Delay(1000);
}
void Filled_Circles_Test(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("Filled Circles", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//	for(uint32_t i = 0; i < 1000; i++)
//	{
//		uint32_t random_num = 0;
//		uint16_t xr = 0;
//		uint16_t yr = 0;
//		uint16_t radiusr = 0;
//		uint16_t colourr = 0;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		xr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		yr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		radiusr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		colourr = random_num;
//
//		xr &= 0x01FF;
//		yr &= 0x01FF;
//		radiusr &= 0x001F;
//		//ili9341_drawpixel(xr, yr, WHITE);
//		ILI9341_Draw_Filled_Circle(xr, yr, radiusr/2, colourr);
//	}
//	HAL_Delay(1000);
}
void Hollow_Rectangles_Test(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("Rectangles", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//	for(uint32_t i = 0; i < 20000; i++)
//	{
//		uint32_t random_num = 0;
//		uint16_t xr = 0;
//		uint16_t yr = 0;
//		uint16_t radiusr = 0;
//		uint16_t colourr = 0;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		xr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		yr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		radiusr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		colourr = random_num;
//
//		xr &= 0x01FF;
//		yr &= 0x01FF;
//		radiusr &= 0x001F;
//		//ili9341_drawpixel(xr, yr, WHITE);
//		ILI9341_Draw_Hollow_Rectangle_Coord(xr, yr, xr+radiusr, yr+radiusr, colourr);
//	}
//	HAL_Delay(1000);
}
void Filled_Rectangles_Test(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("Filled Rectangles", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//	for(uint32_t i = 0; i < 20000; i++)
//	{
//		uint32_t random_num = 0;
//		uint16_t xr = 0;
//		uint16_t yr = 0;
//		uint16_t radiusr = 0;
//		uint16_t colourr = 0;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		xr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		yr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		radiusr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		colourr = random_num;
//
//		xr &= 0x01FF;
//		yr &= 0x01FF;
//		radiusr &= 0x001F;
//		//ili9341_drawpixel(xr, yr, WHITE);
//		ILI9341_Draw_Rectangle(xr, yr, radiusr, radiusr, colourr);
//	}
//	HAL_Delay(1000);
}
void Individual_Pixel_Test(void)
{
	static uint16_t x = 0;
	static uint16_t y = 0;

	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("Slow draw by selecting", 10, 10, BLACK, 1, WHITE);
	ILI9341_Draw_Text("and adressing pixels", 10, 20, BLACK, 1, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(WHITE);

	while (y < 240)
	{
	while ((x < 320) && (y < 240))
	{

		if(x % 2)
		{
			ILI9341_Draw_Pixel(x, y, BLACK);
		}

		x++;
	}

		y++;
		x = 0;
	}

	x = 0;
	y = 0;


	while (y < 240)
	{
	while ((x < 320) && (y < 240))
	{

		if(y % 2)
		{
			ILI9341_Draw_Pixel(x, y, BLACK);
		}

		x++;
	}

		y++;
		x = 0;
	}
	HAL_Delay(2000);
}
void Individual2_Pixel_Test(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Random position and colour", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("500000 pixels", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//
//	for(uint32_t i = 0; i < 500000; i++)
//	{
//		uint32_t random_num = 0;
//		uint16_t xr = 0;
//		uint16_t yr = 0;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		xr = random_num;
//		random_num = HAL_RNG_GetRandomNumber(&hrng);
//		yr = random_num;
//		uint16_t color = HAL_RNG_GetRandomNumber(&hrng);
//
//		xr &= 0x01FF;
//		yr &= 0x01FF;
//		ILI9341_Draw_Pixel(xr, yr, color);
//	}
//	HAL_Delay(2000);
}
void Colour_Test(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Colour gradient", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("Grayscale", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//
//
//	for(uint16_t i = 0; i <= (320); i++)
//	{
//		uint16_t Red = 0;
//		uint16_t Green = 0;
//		uint16_t Blue = 0;
//
//		Red = i/(10);
//		Red <<= 11;
//		Green = i/(5);
//		Green <<= 5;
//		Blue = i/(10);
//
//
//
//		uint16_t RGB_color = Red + Green + Blue;
//		ILI9341_Draw_Rectangle(i, x, 1, 240, RGB_color);
//
//	}
//	HAL_Delay(2000);
}
void Image_Snow_Tiger_Test(void)
{
	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("RGB Picture", 10, 10, BLACK, 1, WHITE);
	ILI9341_Draw_Text("TIGER", 10, 20, BLACK, 1, WHITE);
	HAL_Delay(2000);
	ILI9341_Draw_Image((const char*)snow_tiger, SCREEN_VERTICAL_2);
	ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
	HAL_Delay(10000);
}
void TouchScreen_Test(void)
{
	//put commented section in the task before inf loop
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Touchscreen", 10, 10, BLACK, 2, WHITE);
//	ILI9341_Draw_Text("Touch to draw", 10, 30, BLACK, 2, WHITE);
//	ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//
	if(TP_Touchpad_Pressed())
	{
		uint16_t x_pos = 0;
		uint16_t y_pos = 0;

		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);

		uint16_t position_array[2];

		if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
		{
		x_pos = position_array[0];
		y_pos = position_array[1];
		ILI9341_Draw_Filled_Circle(x_pos, y_pos, 2, BLACK);

		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		char counter_buff[30];
		sprintf(counter_buff, "POS X: %.3d", x_pos);
		ILI9341_Draw_Text(counter_buff, 10, 80, BLACK, 2, WHITE);
		sprintf(counter_buff, "POS Y: %.3d", y_pos);
		ILI9341_Draw_Text(counter_buff, 10, 120, BLACK, 2, WHITE);
		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
		}

		//ILI9341_Draw_Pixel(x_pos, y_pos, BLACK);

	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	Task_action('1'); //sign of life by LED toggle or sending '1' to swo
	HAL_Delay(1000); // remain active for 0.5 second

	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//		ILI9341_Draw_Text("FPS TEST, 40 loop 2 screens", 10, 10, BLACK, 1, WHITE);
	ILI9341_Draw_Text("KOCHAM CIE PATI <3", 10, 10, BLACK, 2, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(ORANGE);
	ILI9341_Draw_Text("KOCHAM CIE PATI <3", 10, 10, BLACK, 2, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(PINK);
	ILI9341_Draw_Text("KOCHAM CIE PATI <3", 10, 10, BLACK, 2, WHITE);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	Task_action('2'); //sign of life by LED toggle or sending '1' to swo
	HAL_Delay(1000); // remain active for 0.5 second
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
