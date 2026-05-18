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
#include "string.h"
#define Hours_toWater 6
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Watering */
osThreadId_t WateringHandle;
const osThreadAttr_t Watering_attributes = {
  .name = "Watering",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LightCtrl */
osThreadId_t LightCtrlHandle;
const osThreadAttr_t LightCtrl_attributes = {
  .name = "LightCtrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ClimateCtrl */
osThreadId_t ClimateCtrlHandle;
const osThreadAttr_t ClimateCtrl_attributes = {
  .name = "ClimateCtrl",
  .stack_size = 128 * 4,
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
volatile int16_t g_lastTemperature = 0;
volatile int16_t g_lastHumidity = 0;
volatile uint8_t uart_rx_byte = 0;

// Runtime-adjustable thresholds
volatile uint16_t SOIL_DRY_THRESHOLD = 2500;
volatile uint16_t WATER_EMPTY_THRESHOLD = 750;
volatile uint16_t LIGHT_LOW_THRESHOLD = 3500;
volatile uint16_t TEMP_COOL_MAX = 24;
volatile uint16_t TEMP_WARM_MAX = 34;
volatile uint16_t HUMID_FAN_LOW = 50;
volatile uint16_t HUMID_FAN_HIGH = 70;

// Settings edit state
volatile uint8_t g_settingsLevel = 0;     // 0 = subsystem pick, 1 = threshold pick
volatile uint8_t g_settingsSubsystem = 0; // 1=Watering, 2=Light, 3=Climate
volatile uint8_t g_editMode = 0;
volatile uint8_t g_editItem = 0;
char g_inputBuf[6];
uint8_t g_inputIdx = 0;
volatile uint8_t g_prevScreen = 0;
volatile uint8_t g_confirmReset = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartWatering(void *argument);
void StartLightCtrl(void *argument);
void StartClimateCtrl(void *argument);
void Callback01(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const char *settingNames[7] = {
    "SoilDry", "WtrLvl", "Light", "HumLow", "HumHi", "TmpCl", "TmpWr"
};

static volatile uint16_t * const thresholdPtrs[7] = {
    &SOIL_DRY_THRESHOLD, &WATER_EMPTY_THRESHOLD, &LIGHT_LOW_THRESHOLD,
    &HUMID_FAN_LOW, &HUMID_FAN_HIGH, &TEMP_COOL_MAX, &TEMP_WARM_MAX
};

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



static void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

static int DHT11_Read(int16_t *temperature, int16_t *humidity) {
    uint8_t data[5] = {0};
    uint16_t wait;
    GPIO_InitTypeDef gpio = {0};

    // --- Start signal: pull LOW >= 18ms (interrupts enabled for HAL_Delay) ---
    gpio.Pin = GPIO_DHT_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_DHT_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin, GPIO_PIN_SET);
    delay_us(30);  // hold HIGH for exactly 30µs

    // Then switch to input with pull-up to wait for DHT response
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIO_DHT_GPIO_Port, &gpio);

    // --- Critical timing: disable interrupts ---
    __disable_irq();

    // Wait for DHT response (LOW 80us)
    wait = 0;
    while (HAL_GPIO_ReadPin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin)) {
        if (++wait > 200) { __enable_irq(); return -1; }
        delay_us(1);
    }
    // Wait for DHT response (HIGH 80us)
    wait = 0;
    while (!HAL_GPIO_ReadPin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin)) {
        if (++wait > 200) { __enable_irq(); return -1; }
        delay_us(1);
    }
    wait = 0;
    while (HAL_GPIO_ReadPin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin)) {
        if (++wait > 200) { __enable_irq(); return -1; }
        delay_us(1);
    }

    // Read 40 bits (5 bytes)
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            wait = 0;
            while (!HAL_GPIO_ReadPin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin)) {
                if (++wait > 200) { __enable_irq(); return -1; }
                delay_us(1);
            }
            delay_us(40);
            if (HAL_GPIO_ReadPin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin)) {
                data[i] = (data[i] << 1) | 1;
                wait = 0;
                while (HAL_GPIO_ReadPin(GPIO_DHT_GPIO_Port, GPIO_DHT_Pin)) {
                    if (++wait > 200) { __enable_irq(); return -1; }
                    delay_us(1);
                }
            } else {
                data[i] = (data[i] << 1) | 0;
            }
        }
    }

    __enable_irq();

    // Checksum
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) return -2;

    *humidity  = data[0];
    *temperature = data[2];
    return 0;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
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

  /* USER CODE BEGIN RTOS_TIMERS */
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

  /* creation of LightCtrl */
  LightCtrlHandle = osThreadNew(StartLightCtrl, NULL, &LightCtrl_attributes);

  /* creation of ClimateCtrl */
  ClimateCtrlHandle = osThreadNew(StartClimateCtrl, NULL, &ClimateCtrl_attributes);

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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim2.Init.Prescaler = 31;
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
  TIM2->PSC = (SystemCoreClock / 1000000) - 1;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CNT = 0;
  HAL_TIM_Base_Start(&htim2);
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
  HAL_GPIO_WritePin(GPIO_Water_GPIO_Port, GPIO_Water_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_DHT_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_Water_Pin */
  GPIO_InitStruct.Pin = GPIO_Water_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_Water_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_DHT_Pin LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_DHT_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	HAL_GPIO_WritePin(GPIOA, GPIO_Water_Pin, GPIO_PIN_SET);   // relay OFF at boot
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void uart_msg(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

static void print_screen_name(void) {
    switch (currentScreen) {
        case 0: uart_msg("Water Subsystem\r\n"); break;
        case 1: uart_msg("Light Subsystem\r\n"); break;
        case 2: uart_msg("Climate Subsystem\r\n"); break;
        case 3: uart_msg("Settings\r\n"); break;
    }
}

static void settings_commit(void) {
    g_inputBuf[g_inputIdx] = '\0';
    uint16_t val = 0;
    for (uint8_t i = 0; i < g_inputIdx; i++)
        val = val * 10 + (g_inputBuf[i] - '0');
    if (val > 100) val = 100;
    uint16_t raw = val;
    switch (g_editItem) {
        case 1:
            raw = (uint16_t)((100u - val) * 4095u / 100u);
            SOIL_DRY_THRESHOLD = raw; break;
        case 2:
        case 3:
            raw = (uint16_t)(val * 4095u / 100u);
            if (g_editItem == 2) WATER_EMPTY_THRESHOLD = raw;
            else LIGHT_LOW_THRESHOLD = raw;
            break;
        case 4: HUMID_FAN_LOW = val; break;
        case 5: HUMID_FAN_HIGH = val; break;
        case 6: TEMP_COOL_MAX = val; break;
        case 7: TEMP_WARM_MAX = val; break;
    }
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "%s set to %u\r\n", settingNames[g_editItem - 1], val);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
    g_editMode = 0;
    g_editItem = 0;
    g_settingsLevel = 0;
    g_settingsSubsystem = 0;
}

static void settings_reset_defaults(void) {
    SOIL_DRY_THRESHOLD = 2500;
    WATER_EMPTY_THRESHOLD = 750;
    LIGHT_LOW_THRESHOLD = 3500;
    TEMP_COOL_MAX = 24;
    TEMP_WARM_MAX = 34;
    HUMID_FAN_LOW = 50;
    HUMID_FAN_HIGH = 70;
    uart_msg("Defaults restored\r\n");
}

static void settings_start_edit(uint8_t item) {
    g_editItem = item;
    g_editMode = 1;
    g_inputIdx = 0;
    g_inputBuf[0] = '\0';
    uint16_t cur = *thresholdPtrs[item - 1];
    char buf[32];
    if (item <= 3) {
        uint16_t pct = (item == 1)
            ? 100 - (cur * 100 / 4095)
            : (cur * 100 / 4095);
        int len = snprintf(buf, sizeof(buf), "%s: %u%%\r\n", settingNames[item - 1], pct);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
    } else {
        int len = snprintf(buf, sizeof(buf), "%s: %u\r\n", settingNames[item - 1], cur);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (currentScreen == 3 && g_editMode) {
            if (uart_rx_byte >= '0' && uart_rx_byte <= '9') {
                if (g_inputIdx < 5) {
                    g_inputBuf[g_inputIdx++] = uart_rx_byte;
                }
            } else if (uart_rx_byte == '\b' || uart_rx_byte == 0x7F) {
                if (g_inputIdx > 0) g_inputIdx--;
            } else if (uart_rx_byte == '\r' || uart_rx_byte == '\n') {
                if (g_inputIdx > 0) settings_commit();
            } else if (uart_rx_byte == 0x1B) {
                uart_msg("Cancelled\r\n");
                g_editMode = 0;
                g_editItem = 0;
                g_settingsLevel = 0;
                g_settingsSubsystem = 0;
            }
        } else if (currentScreen == 3 && !g_editMode) {
            if (g_confirmReset) {
                if (uart_rx_byte == 'y' || uart_rx_byte == 'Y') {
                    settings_reset_defaults();
                    g_confirmReset = 0;
                } else {
                    uart_msg("Reset cancelled\r\n");
                    g_confirmReset = 0;
                }
            } else if (g_settingsLevel == 0) {
                if (uart_rx_byte == '1') {
                    g_settingsSubsystem = 1;
                    g_settingsLevel = 1;
                } else if (uart_rx_byte == '2') {
                    settings_start_edit(3);
                } else if (uart_rx_byte == '3') {
                    g_settingsSubsystem = 3;
                    g_settingsLevel = 1;
                } else if (uart_rx_byte == '4') {
                    g_confirmReset = 1;
                } else if (uart_rx_byte == 's' || uart_rx_byte == 'S'
                        || uart_rx_byte == 0x1B) {
                    uart_msg("Exiting Settings\r\n");
                    currentScreen = g_prevScreen;
                }
            } else if (g_settingsLevel == 1) {
                if (g_settingsSubsystem == 1) {
                    if (uart_rx_byte == '1') settings_start_edit(1);
                    else if (uart_rx_byte == '2') settings_start_edit(2);
                } else if (g_settingsSubsystem == 3) {
                    if (uart_rx_byte >= '1' && uart_rx_byte <= '4')
                        settings_start_edit(uart_rx_byte - '0' + 3);
                }
                if (uart_rx_byte == 's' || uart_rx_byte == 'S'
                        || uart_rx_byte == 0x1B || uart_rx_byte == '\b') {
                    g_settingsLevel = 0;
                    g_settingsSubsystem = 0;
                }
            }
        } else {
            uint8_t prevScreen = currentScreen;
            if (uart_rx_byte == 'd' || uart_rx_byte == 'D')
                currentScreen = (currentScreen >= 2) ? 0 : (currentScreen + 1);
            else if (uart_rx_byte == 'a' || uart_rx_byte == 'A')
                currentScreen = (currentScreen == 0) ? 2 : (currentScreen - 1);
            else if (uart_rx_byte == 's' || uart_rx_byte == 'S') {
                g_prevScreen = currentScreen;
                currentScreen = 3;
            }
            if (currentScreen != prevScreen) print_screen_name();
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
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
  osDelay(100);
  LCD_Init(&hi2c1);
  lcdReady = 1;

  osMutexAcquire(lcdMutexHandle, osWaitForever);
  LCD_SetCursor(0, 0);
  LCD_Print("  Greenhouse    ");
  LCD_SetCursor(1, 0);
  LCD_Print("  Starting...   ");
  osMutexRelease(lcdMutexHandle);
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);  
  
  osDelay(2000);

  uint8_t lastScreen = 255; // force clear on first render

  for(;;)
  {
    if (currentScreen != lastScreen) {
        // screen just changed � clear display
        osMutexAcquire(lcdMutexHandle, osWaitForever);
        LCD_Clear();
        osMutexRelease(lcdMutexHandle);
        lastScreen = currentScreen;
    }

    osMutexAcquire(lcdMutexHandle, osWaitForever);

    if (currentScreen == 0) {
        char line0[17], line1[17];
        uint16_t soilPct = 100 - (g_lastSoilValue * 100 / 4095);
        snprintf(line0, sizeof(line0), "Soil:%3u%% %s",
                 soilPct,
                 g_lastSoilValue > SOIL_DRY_THRESHOLD ? "DRY" : "OK ");
        LCD_SetCursor(0, 0);
        LCD_Print(line0);
        LCD_SetCursor(1, 0);
        uint16_t tankPct = g_lastWaterLevel * 100 / 4095;
        if (tankEmpty) {
            snprintf(line1, sizeof(line1), "Tank:%3u%% EMPTY", tankPct);
        } else if (!wateringAllowed) {
            snprintf(line1, sizeof(line1), "Tank:%3u%% COOLD", tankPct);
        } else {
            snprintf(line1, sizeof(line1), "Tank:%3u%% OK   ", tankPct);
        }
        LCD_Print(line1);
    } else if (currentScreen == 1) {
        char line0[17], line1[17];
        uint16_t lightPct = g_lastLightValue * 100 / 4095;
        snprintf(line0, sizeof(line0), "Light:%3u%%      ", lightPct);
        snprintf(line1, sizeof(line1), "Strip: %s       ",
                 g_lastLightValue < LIGHT_LOW_THRESHOLD ? "ON " : "OFF");
        LCD_SetCursor(0, 0);
        LCD_Print(line0);
        LCD_SetCursor(1, 0);
        LCD_Print(line1);
    } else if (currentScreen == 2) {
        char line0[17], line1[17];
        const char *fan;
        if (g_lastHumidity <= HUMID_FAN_LOW)
            fan = "OFF";
        else if (g_lastHumidity <= HUMID_FAN_HIGH)
            fan = "50%";
        else
            fan = "100%";
        snprintf(line0, sizeof(line0), "Temp:%dC          ", g_lastTemperature);
        snprintf(line1, sizeof(line1), "Hum:%d%% F:%-4s ", g_lastHumidity, fan);
        LCD_SetCursor(0, 0);
        LCD_Print(line0);
        LCD_SetCursor(1, 0);
        LCD_Print(line1);
    } else if (currentScreen == 3) {
        char line0[17], line1[17];
        if (g_confirmReset) {
            snprintf(line0, sizeof(line0), "%-16s", "Rst defaults?");
            snprintf(line1, sizeof(line1), "%-16s", "Y / N");
        } else if (g_editMode) {
            uint16_t cur = *thresholdPtrs[g_editItem - 1];
            char tmp[17];
            if (g_editItem <= 3) {
                uint16_t pct = (g_editItem == 1)
                    ? 100 - (cur * 100 / 4095)
                    : (cur * 100 / 4095);
                snprintf(tmp, sizeof(tmp), "%s:%u%%", settingNames[g_editItem - 1], pct);
            } else {
                snprintf(tmp, sizeof(tmp), "%s:%u", settingNames[g_editItem - 1], cur);
            }
            snprintf(line0, sizeof(line0), "%-16s", tmp);
            g_inputBuf[g_inputIdx] = '\0';
            snprintf(tmp, sizeof(tmp), "%s>_", g_inputIdx == 0 ? "" : g_inputBuf);
            snprintf(line1, sizeof(line1), "%-16s", tmp);
        } else if (g_settingsLevel == 0) {
            snprintf(line0, sizeof(line0), "%-16s", "1-Water 2-Light");
            snprintf(line1, sizeof(line1), "%-16s", "3-Climate 4-RST");
        } else if (g_settingsLevel == 1) {
            if (g_settingsSubsystem == 1) {
                snprintf(line0, sizeof(line0), "%-16s", "1-SoilDry");
                snprintf(line1, sizeof(line1), "%-16s", "2-WtrLvl");
            } else if (g_settingsSubsystem == 3) {
                snprintf(line0, sizeof(line0), "%-16s", "1-HumLw 2-HumHi");
                snprintf(line1, sizeof(line1), "%-16s", "3-TmpCl 4-TmpWr");
            }
        }
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
    /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)buffer1, len1, 100);
    osDelay(1000);

    if (waterLevel < WATER_EMPTY_THRESHOLD) {
        tankEmpty = 1;
        /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)"TANK EMPTY\r\n", 12, 100);
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
        /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
        osDelay(1000);
    }
  }
  /* USER CODE END StartWatering */
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
		/* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)dbg, dlen, 100);

    if (lightValue < LIGHT_LOW_THRESHOLD) {
			  duty = (uint32_t)(99.0f * (1.0f - (lightValue / 5000.0f)));
        if (duty > 99) duty = 99;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
			int len = snprintf(buffer, sizeof(buffer), "duty: %u\r\n", duty);
    /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
       
    } else {
         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }

    int len = snprintf(buffer, sizeof(buffer), "Light: %u\r\n", lightValue);
    /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);

    osDelay(500);
  }
  /* USER CODE END StartLightCtrl */
}

/* USER CODE BEGIN Header_StartClimateCtrl */
/**
* @brief Function implementing the ClimateCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartClimateCtrl */
void StartClimateCtrl(void *argument)
{
  /* USER CODE BEGIN StartClimateCtrl */
  int16_t temperature, humidity;
  char buf[40];
  int ret;

  while (!lcdReady) { osDelay(10); }
  osDelay(3000);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  for(;;)
  {
    ret = DHT11_Read(&temperature, &humidity);
    if (ret == 0) {
        g_lastTemperature = temperature;
        g_lastHumidity = humidity;
        int len = snprintf(buf, sizeof(buf), "Temp:%dC Hum:%d%%\r\n", temperature, humidity);
        /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);

        if (humidity <= HUMID_FAN_LOW) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)"Fan: OFF\r\n", 10, 100);
        } else if (humidity <= HUMID_FAN_HIGH) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 49);
            /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)"Fan: 50%\r\n", 10, 100);
        } else {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 99);
            /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)"Fan: 100%\r\n", 11, 100);
        }
    } else {
        /* Debug Statement */ //HAL_UART_Transmit(&huart2, (uint8_t*)"DHT11 FAIL\r\n", 12, 100);
    }
    osDelay(4000);
  }
  /* USER CODE END StartClimateCtrl */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
  wateringAllowed = 1;
  /* USER CODE END Callback01 */
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
