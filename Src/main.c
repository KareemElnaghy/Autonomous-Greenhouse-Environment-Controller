/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdio.h"
#include "lcd_i2c.h"
#include "queue.h"
#define SOIL_DRY_THRESHOLD 2500
#define Hours_toWater 6
#define WATER_EMPTY_THRESHOLD 750
#define LIGHT_LOW_THRESHOLD   3500
#define LIGHT_CHECK_INTERVAL  5000

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Watering */
osThreadId_t WateringHandle;
const osThreadAttr_t Watering_attributes = {
  .name = "Watering",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Fertilizer */
osThreadId_t FertilizerHandle;
const osThreadAttr_t Fertilizer_attributes = {
  .name = "Fertilizer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LightCtrl */
osThreadId_t LightCtrlHandle;
const osThreadAttr_t LightCtrl_attributes = {
  .name = "LightCtrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RQ */
osMessageQueueId_t RQHandle;
const osMessageQueueAttr_t RQ_attributes = {
  .name = "RQ"
};
/* Definitions for Timerwater */
osTimerId_t TimerwaterHandle;
const osTimerAttr_t Timerwater_attributes = {
  .name = "Timerwater"
};
/* Definitions for TimerFertilizer */
osTimerId_t TimerFertilizerHandle;
const osTimerAttr_t TimerFertilizer_attributes = {
  .name = "TimerFertilizer"
};
/* Definitions for ScreenToggleTimer */
osTimerId_t ScreenToggleTimerHandle;
const osTimerAttr_t ScreenToggleTimer_attributes = {
  .name = "ScreenToggleTimer"
};
/* Definitions for WorkingSemaphore */
osSemaphoreId_t WorkingSemaphoreHandle;
const osSemaphoreAttr_t WorkingSemaphore_attributes = {
  .name = "WorkingSemaphore"
};
/* USER CODE BEGIN PV */
volatile uint8_t wateringAllowed = 1;
volatile uint8_t tankEmpty  = 0;
volatile uint8_t lcdReady = 0;
volatile uint8_t currentScreen = 0;  // 0 = water screen, 1 = light screen
osMutexId_t lcdMutexHandle;
volatile uint16_t g_lastSoilValue  = 0;
volatile uint16_t g_lastWaterLevel = 0;
volatile uint16_t g_lastLightValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartWatering(void *argument);
void StartFertilizer(void *argument);
void StartLightCtrl(void *argument);
void Callback01(void *argument);
void Callback02(void *argument);
void ScreenToggleCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        uint16_t soilValue = (uint16_t)HAL_ADC_GetValue(&hadc1);
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(RQHandle, &soilValue, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

uint16_t ReadWaterLevel(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);

    // switch back to soil moisture
    sConfig.Channel = ADC_CHANNEL_6;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    return val;
}

uint16_t ReadLightLevel(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_5;   // PA0 = IN5 on L432KC
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);

    // switch back to soil moisture channel
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    return val;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	lcdMutexHandle = osMutexNew(NULL);

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of WorkingSemaphore */
  WorkingSemaphoreHandle = osSemaphoreNew(1, 1, &WorkingSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Timerwater */
  TimerwaterHandle = osTimerNew(Callback01, osTimerOnce, NULL, &Timerwater_attributes);

  /* creation of TimerFertilizer */
  TimerFertilizerHandle = osTimerNew(Callback02, osTimerPeriodic, NULL, &TimerFertilizer_attributes);

  /* creation of ScreenToggleTimer */
  ScreenToggleTimerHandle = osTimerNew(ScreenToggleCallback, osTimerPeriodic, NULL, &ScreenToggleTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	osTimerStart(ScreenToggleTimerHandle, pdMS_TO_TICKS(5000));
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of RQ */
  RQHandle = osMessageQueueNew (16, sizeof(uint16_t), &RQ_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Watering */
  WateringHandle = osThreadNew(StartWatering, NULL, &Watering_attributes);

  /* creation of Fertilizer */
  FertilizerHandle = osThreadNew(StartFertilizer, NULL, &Fertilizer_attributes);

  /* creation of LightCtrl */
  LightCtrlHandle = osThreadNew(StartLightCtrl, NULL, &LightCtrl_attributes);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_Fertilizer_Pin|GPIO_Water_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_Light_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_Fertilizer_Pin GPIO_Water_Pin */
  GPIO_InitStruct.Pin = GPIO_Fertilizer_Pin|GPIO_Water_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Light_Pin LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_Light_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	HAL_GPIO_WritePin(GPIOA, GPIO_Water_Pin, GPIO_PIN_SET);   // relay OFF at boot

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  osDelay(100);
  LCD_Init(&hi2c1);
  lcdReady = 1;

  osMutexAcquire(lcdMutexHandle, osWaitForever);
  LCD_SetCursor(0, 0);
  LCD_Print("  Greenhouse    ");
  LCD_SetCursor(1, 0);
  LCD_Print("  Starting...   ");
  osMutexRelease(lcdMutexHandle);

  osDelay(2000);

  uint8_t lastScreen = 255; // force clear on first render

  for(;;)
  {
    if (currentScreen != lastScreen) {
        // screen just changed — clear display
        osMutexAcquire(lcdMutexHandle, osWaitForever);
        LCD_Clear();
        osMutexRelease(lcdMutexHandle);
        lastScreen = currentScreen;
    }

    osMutexAcquire(lcdMutexHandle, osWaitForever);

    if (currentScreen == 0) {
        char line0[17], line1[17];
        snprintf(line0, sizeof(line0), "Soil:%-5u %s",
                 g_lastSoilValue,
                 g_lastSoilValue > SOIL_DRY_THRESHOLD ? "DRY " : "OK  ");
        LCD_SetCursor(0, 0);
        LCD_Print(line0);
        LCD_SetCursor(1, 0);
        if (tankEmpty) {
            LCD_Print("Tank: EMPTY     ");
        } else if (!wateringAllowed) {
            LCD_Print("Tank: COOLDOWN  ");
        } else {
            snprintf(line1, sizeof(line1), "Tank:%-5u OK   ", g_lastWaterLevel);
            LCD_Print(line1);
        }
    } else {
        char line0[17], line1[17];
        snprintf(line0, sizeof(line0), "Lux: %-5u      ", g_lastLightValue);
        snprintf(line1, sizeof(line1), "Strip: %s       ",
                 g_lastLightValue < LIGHT_LOW_THRESHOLD ? "ON " : "OFF");
        LCD_SetCursor(0, 0);
        LCD_Print(line0);
        LCD_SetCursor(1, 0);
        LCD_Print(line1);
    }

    osMutexRelease(lcdMutexHandle);
    osDelay(500); // refresh every 500ms
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartWatering */
/**
* @brief Function implementing the Watering thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWatering */
void StartWatering(void *argument)
{
  /* USER CODE BEGIN StartWatering */
  while (!lcdReady) { osDelay(10); }
  osDelay(2500);

  uint16_t soilvalue;
  uint16_t waterLevel;
  char buffer[20];
  char buffer1[20];

  for(;;)
  {
    HAL_ADC_Start_IT(&hadc1);
    osMessageQueueGet(RQHandle, &soilvalue, NULL, osWaitForever);
    g_lastSoilValue = soilvalue;

    waterLevel = ReadWaterLevel();
    g_lastWaterLevel = waterLevel;

    int len1 = snprintf(buffer1, sizeof(buffer1), "water: %u\r\n", waterLevel);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer1, len1, 100);
    osDelay(1000);

    if (waterLevel < WATER_EMPTY_THRESHOLD) {
        tankEmpty = 1;
        HAL_UART_Transmit(&huart2, (uint8_t*)"TANK EMPTY\r\n", 12, 100);
    } else {
        tankEmpty = 0;
    }

    if (soilvalue > SOIL_DRY_THRESHOLD && wateringAllowed && !tankEmpty) {
        if (osSemaphoreAcquire(WorkingSemaphoreHandle, 0) == osOK) {
            wateringAllowed = 0;
            osDelay(6000);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            osDelay(5000);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            osSemaphoreRelease(WorkingSemaphoreHandle);
            osTimerStart(TimerwaterHandle, pdMS_TO_TICKS(10000));
        }
        int len = snprintf(buffer, sizeof(buffer), "Soil: %u\r\n", soilvalue);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
        osDelay(1000);
    }
  }
  /* USER CODE END StartWatering */
}

/* USER CODE BEGIN Header_StartFertilizer */
/**
* @brief Function implementing the Fertilizer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFertilizer */
void StartFertilizer(void *argument)
{
  /* USER CODE BEGIN StartFertilizer */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFertilizer */
}

/* USER CODE BEGIN Header_StartLightCtrl */
/**
* @brief Function implementing the LightCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightCtrl */
void StartLightCtrl(void *argument)
{
  /* USER CODE BEGIN StartLightCtrl */
  while (!lcdReady) { osDelay(10); }

  uint16_t lightValue;
  char buffer[40];
	uint32_t duty;
	float psc;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  for(;;)
  {
    lightValue = ReadLightLevel();
    g_lastLightValue = lightValue;
			psc = (320000/100) - 1 ;
	// set a prescaler value for the intended timer
	  __HAL_TIM_SET_PRESCALER(&htim1,psc);
		
		char dbg[30];
		int dlen = snprintf(dbg, sizeof(dbg), "LightRaw: %u\r\n", lightValue);
		HAL_UART_Transmit(&huart2, (uint8_t*)dbg, dlen, 100);

    if (lightValue < LIGHT_LOW_THRESHOLD) {
			  duty = (uint32_t)(99.0f * (1.0f - (lightValue / 5000.0f)));
        if (duty > 99) duty = 99;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
			int len = snprintf(buffer, sizeof(buffer), "duty: %u\r\n", duty);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
       
    } else {
         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }

    int len = snprintf(buffer, sizeof(buffer), "Light: %u\r\n", lightValue);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);

    osDelay(500);
  }
  /* USER CODE END StartLightCtrl */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
		wateringAllowed = 1;
  /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void *argument)
{
  /* USER CODE BEGIN Callback02 */

  /* USER CODE END Callback02 */
}

/* ScreenToggleCallback function */
void ScreenToggleCallback(void *argument)
{
  /* USER CODE BEGIN ScreenToggleCallback */
    currentScreen = (currentScreen == 0) ? 1 : 0;
  /* USER CODE END ScreenToggleCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
