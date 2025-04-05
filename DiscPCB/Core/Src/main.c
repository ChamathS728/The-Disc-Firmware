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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DRV8825.h"
#include "discStateMachine.h"
#include "stdio.h"
#include "threadFlags.h"
#include "SPI_Comms.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define COMMAND_OVER_USB 1	// Uncomment if we want to receive data from USB

#define USE_BUZZER 1		// Uncomment if we don't want the buzzer to sound
#define BUZZ_ARR 40000
#define BUZZ_PSC 9
#define BUZZ_CHANNEL TIM_CHANNEL_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CORDIC_HandleTypeDef hcordic;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for strelkaCommsTas */
osThreadId_t strelkaCommsTasHandle;
const osThreadAttr_t strelkaCommsTas_attributes = {
  .name = "strelkaCommsTas",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for powerSenseTask */
osThreadId_t powerSenseTaskHandle;
const osThreadAttr_t powerSenseTask_attributes = {
  .name = "powerSenseTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for stepperCtrlTask */
osThreadId_t stepperCtrlTaskHandle;
const osThreadAttr_t stepperCtrlTask_attributes = {
  .name = "stepperCtrlTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for stateMachineTas */
osThreadId_t stateMachineTasHandle;
const osThreadAttr_t stateMachineTas_attributes = {
  .name = "stateMachineTas",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for decodeUSBTask */
osThreadId_t decodeUSBTaskHandle;
const osThreadAttr_t decodeUSBTask_attributes = {
  .name = "decodeUSBTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for adcBattBuff */
osMessageQueueId_t adcBattBuffHandle;
const osMessageQueueAttr_t adcBattBuff_attributes = {
  .name = "adcBattBuff"
};
/* Definitions for usbBuff */
osMessageQueueId_t usbBuffHandle;
const osMessageQueueAttr_t usbBuff_attributes = {
  .name = "usbBuff"
};
/* Definitions for motorData */
osMessageQueueId_t motorDataHandle;
const osMessageQueueAttr_t motorData_attributes = {
  .name = "motorData"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
void strelkaCommsFn(void *argument);
void powerSenseFn(void *argument);
void stepperCtrlFn(void *argument);
void stateMachineFn(void *argument);
void decodeUSBFn(void *argument);

/* USER CODE BEGIN PFP */
uint8_t rxDiscSPI[PACKET_SIZE_STRELKA_RX];
uint8_t txDiscSPI[PACKET_SIZE_STRELKA_RX];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TIM_HandleTypeDef* EncoderTimer = &htim1;
TIM_HandleTypeDef* MillisTimer = &htim2;
TIM_HandleTypeDef* PWMTimer = &htim3;
TIM_HandleTypeDef* PWMStopTimer = &htim4;

SPI_HandleTypeDef* StrelkaV2SPI = &hspi2;

// Initialise buffers
uint8_t txBuff[3] = {'x', 'y', 'z'};
uint8_t rxBuff[3];

DeviceStatus_t discStatus = {
		.currentPosition = 0,
		.currentTime = 0,
		.targetPosition = 0,
		.isMoving = 0
};

uint8_t isTargetNew = 1;
int numOfRevolutions = 0;

// Overwrite _write method to use printf for sending to computer
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    //__io_putchar(*ptr++);
	  ITM_SendChar(*ptr++);
  }
  return len;
}
// Define RxHandler to receive data over USB
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	// Define CDC RxHandler
	#ifdef COMMAND_OVER_USB
    	CDC_Transmit_FS(Buf, Len);
	#endif
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	osThreadFlagsSet(powerSenseTaskHandle, isADCDone);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
	// Pull header straight from receive buffer
	PacketHeader_t* header = (PacketHeader_t*) hspi->pRxBuffPtr;

	// Get packet type from struct and go through each possibility
	switch (header->packetType) {
		case PACKET_TYPE_MOVE:
			// Decode move packet
			uint16_t targetPos = decodeMovePacket();

			char printBuff[64];
			sprintf(printBuff, "%lu\n", targetPos);
			printf(printBuff);

			// Notify stepper thread that target position has changed
			osThreadFlagsSet(stepperCtrlTaskHandle, isTargetNew);
			break;
		case PACKET_TYPE_RETRACT_FULL:
			// FIXME - Overwrite target position
			printf("Retract Packet Received\n");

			// Set target position in deviceStatusStruct

			// Send back device status

			// Notify stepper thread that target position has changed
			osThreadFlagsSet(stepperCtrlTaskHandle, isTargetNew);
			break;
		case PACKET_TYPE_EXTEND_FULL:
			// FIXME - Overwrite target position
			printf("Extend Packet Received\n");
			// Set target position in deviceStatusStruct

			// Send back device status

			// Notify stepper thread that target position has changed
			osThreadFlagsSet(stepperCtrlTaskHandle, isTargetNew);
			break;
		default:
			break;
	}

	// Restart SPI comms
	HAL_SPI_Receive_IT(StrelkaV2SPI, rxDiscSPI, sizeof(rxDiscSPI));
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
	printf(rxBuff);
	HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SPI2_CS_Pin) {
		HAL_SPI_TransmitReceive_IT(StrelkaV2SPI, txDiscSPI, rxDiscSPI, sizeof(rxDiscSPI));
	}
}

uint32_t millis(void) {
	return MillisTimer->Instance->CNT;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_CORDIC_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

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

  /* Create the queue(s) */
  /* creation of adcBattBuff */
  adcBattBuffHandle = osMessageQueueNew (4, sizeof(uint16_t), &adcBattBuff_attributes);

  /* creation of usbBuff */
  usbBuffHandle = osMessageQueueNew (64, sizeof(uint16_t), &usbBuff_attributes);

  /* creation of motorData */
  motorDataHandle = osMessageQueueNew (64, sizeof(uint16_t), &motorData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of strelkaCommsTas */
  strelkaCommsTasHandle = osThreadNew(strelkaCommsFn, NULL, &strelkaCommsTas_attributes);

  /* creation of powerSenseTask */
  powerSenseTaskHandle = osThreadNew(powerSenseFn, NULL, &powerSenseTask_attributes);

  /* creation of stepperCtrlTask */
  stepperCtrlTaskHandle = osThreadNew(stepperCtrlFn, NULL, &stepperCtrlTask_attributes);

  /* creation of stateMachineTas */
  stateMachineTasHandle = osThreadNew(stateMachineFn, NULL, &stateMachineTas_attributes);

  /* creation of decodeUSBTask */
  decodeUSBTaskHandle = osThreadNew(decodeUSBFn, NULL, &decodeUSBTask_attributes);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 20000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, ERROR_LED_Pin|DEBUG_LED_Pin|MTR_DECAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MTR_NENBL_Pin|MTR_NSLP_Pin|MTR_DIR_Pin|MTR_M0_Pin
                          |MTR_M1_Pin|MTR_M2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MTR_NRST_GPIO_Port, MTR_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ERROR_LED_Pin DEBUG_LED_Pin MTR_DECAY_Pin */
  GPIO_InitStruct.Pin = ERROR_LED_Pin|DEBUG_LED_Pin|MTR_DECAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin MTR_NHOME_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|MTR_NHOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MTR_NFLT_Pin */
  GPIO_InitStruct.Pin = MTR_NFLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MTR_NFLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_NENBL_Pin MTR_DIR_Pin MTR_M0_Pin MTR_M1_Pin
                           MTR_M2_Pin */
  GPIO_InitStruct.Pin = MTR_NENBL_Pin|MTR_DIR_Pin|MTR_M0_Pin|MTR_M1_Pin
                          |MTR_M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MTR_NSLP_Pin */
  GPIO_InitStruct.Pin = MTR_NSLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MTR_NSLP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MTR_NRST_Pin */
  GPIO_InitStruct.Pin = MTR_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MTR_NRST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_strelkaCommsFn */
/**
  * @brief  Function implementing the strelkaComms thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_strelkaCommsFn */
void strelkaCommsFn(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */

  // Initialise buffers
//  uint8_t txBuff[3] = {"a", "b", "\n"};
//  uint8_t rxBuff[3];

  // Start off receive with interrupts
  HAL_SPI_Receive_IT(StrelkaV2SPI, rxDiscSPI, sizeof(rxDiscSPI));

  // Send initial message
  HAL_SPI_Transmit_IT(StrelkaV2SPI, txBuff, 3);

  /* Infinite loop */
  for(;;)
  {
//	  HAL_SPI_TransmitReceive_IT(StrelkaV2SPI, txBuff, rxBuff, 3);
	  HAL_SPI_Receive_IT(StrelkaV2SPI, rxDiscSPI, sizeof(rxDiscSPI));
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_powerSenseFn */
/**
* @brief Function implementing the powerSense thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_powerSenseFn */
void powerSenseFn(void *argument)
{
  /* USER CODE BEGIN powerSenseFn */
	// Initialise variables
	uint16_t buffADC[4] = {0,0,0,0};
	uint32_t mtr_A_I = 0;
	uint32_t mtr_B_I = 0;
	uint32_t batt_V = 0;
	uint32_t batt_I = 0;

	// Start up ADCs
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, buffADC, 4);

  /* Infinite loop */
  for(;;)
  {
	  // Task waits for ADC callback to run
	  osThreadFlagsWait(isADCDone, osFlagsWaitAny, osWaitForever);

	  char blah[64];
//		  sprintf(blah, "%d,%d,%d,%d\n", mtr_A_I, mtr_B_I, batt_V, batt_I);
//		  sprintf(blah, "%d,%d,%d,%d\n",buffADC[0],buffADC[1],buffADC[2],buffADC[3]);
//		  printf(blah);

	  // Restart ADC
	  HAL_ADC_Start_DMA(&hadc1, buffADC, 4);

	  // Convert data from each channel to actual units
    osDelay(100);
  }
  /* USER CODE END powerSenseFn */
}

/* USER CODE BEGIN Header_stepperCtrlFn */
/**
* @brief Function implementing the stepperCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stepperCtrlFn */
void stepperCtrlFn(void *argument)
{
  /* USER CODE BEGIN stepperCtrlFn */
  /* Infinite loop */

#ifdef USE_BUZZER
	// Wait until buzzer complete
	osThreadFlagsWait(buzzerDone, osFlagsWaitAny, osWaitForever);
#endif
	// Start up stepper handle
	stepperIO_t sIO = {
			.M0Port 		= MTR_M0_GPIO_Port,
			.M0Pin 			= MTR_M0_Pin,
			.M1Port 		= MTR_M1_GPIO_Port,
			.M1Pin 			= MTR_M1_Pin,
			.M2Port 		= MTR_M2_GPIO_Port,
			.M2Pin 			= MTR_M2_Pin,
			.decayPort 		= MTR_DECAY_GPIO_Port,
			.decayPin 		= MTR_DECAY_Pin,
			.dirPort 		= MTR_DIR_GPIO_Port,
			.dirPin 		= MTR_DIR_Pin,
			.nEnablePort 	= MTR_NENBL_GPIO_Port,
			.nEnablePin 	= MTR_NENBL_Pin,
			.nFaultPort 	= MTR_NFLT_GPIO_Port,
			.nFaultPort 	= MTR_NFLT_Pin,
			.nHomePort 		= MTR_NHOME_GPIO_Port,
			.nHomePin	 	= MTR_NHOME_Pin,
			.nResetPort 	= MTR_NRST_GPIO_Port,
			.nResetPin 		= MTR_NRST_Pin,
			.nSleepPort 	= MTR_NSLP_GPIO_Port,
			.nSleepPin 		= MTR_NSLP_Pin
	};

	stepperRotInfo_t sRot = {
			.PWMPtr 		= PWMTimer,
			.PWMStopPtr 	= PWMStopTimer,
			.driveRes 		= REV_8,
			.driverSteps 	= 0,
			.encPPR 		= 1000,
			.encPtr 		= EncoderTimer,
			.encPulses 		= 0,
			.maxAngle 		= 90.0,
			.minAngle 		= 0.0
	};

	stepperConfig_t sCfg = {
			.moveProfile 	= MOVE_TRAP,
			.stepRes 		= MICROSTEP_8,
			.stepperDir 	= 0
	};

	stepperHandle_t* mtrHandle = DRV_init(&sCfg, &sIO, &sRot);
	DRV_wakeup(mtrHandle);
	DRV_start(mtrHandle);
//	__HAL_TIM_SET_PRESCALER(mtrHandle->rotInfo->PWMPtr, 8);
	DRV_set_pulse_freq(mtrHandle, 300);

	DRV_move_steps(mtrHandle, 400, 1); // 1 means anticlockwise as of 31/01/25
	osDelay(2000);
	DRV_move_steps(mtrHandle, 400, 0);
	osDelay(2000);
	DRV_sleep(mtrHandle);

	// Set up control loop parameters
//	PIDController_t PID;
//	float Kp = 1;
//	float Ki = 0;
//	float Kd = 0;
//	float dt = 10;
//
//	float output_min = 0.0f;
//	float output_min = 1.0f;
//
//	float alpha = 1;
//
//	PID_Init(PID, Kp, Ki, Kd, dt, alpha);

  for(;;)
  {
//	  // Get encoder
//	  uint32_t currentEnc = mtrHandle->rotInfo->encPtr->Instance->CNT;
//
//	  // Get absolute angular position
//	  if (numOfRevolutions > 0) {
//		  mtrHandle->rotInfo->encPulses = (int16_t) (mtrHandle->rotInfo->encPtr->Instance->ARR * numOfRevolutions + currentEnc);
//	  }
//	  else if (numOfRevolutions < 0) {
//		  mtrHandle->rotInfo->encPulses = (int16_t) mtrHandle->rotInfo->encPtr->Instance->ARR * numOfRevolutions * -1 - (int16_t) (mtrHandle->rotInfo->encPtr->Instance->ARR - currentEnc);
//	  }
//	  else {
//		  mtrHandle->rotInfo->encPulses = (int16_t) currentEnc;
//	  }
//
//	  discStatus.currentPosition = mtrHandle->rotInfo->encPulses;
//
//	  // Get error based on this
//	  float error = (float) (discStatus.targetPosition - discStatus.currentPosition);
//
//	  // Run PID controller -> outputs velocity
//	  float outPID = PID_Update(&PID, error);
//
//	  // Get number of steps for this iteration of the control loop
//	  int loopSteps =  outPID * PID.dt;
//	  uint8_t dir = 0;
//	  if (loopSteps < 0) {
//		  dir = 0;
//	  }
//	  DRV_move_steps(mtrHandle, (uint16_t) loopSteps, dir);

//	  HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
//	  if (HAL_GPIO_ReadPin(MTR_NFLT_GPIO_Port, MTR_NFLT_Pin) == GPIO_PIN_SET) {
//		  HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
//	  }
//	  HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
//	  HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
//	  vTaskSuspendAll();
//	  DRV_move_steps(blah, 0, 0);
//	  xTaskResumeAll();
	  uint32_t encoderVal = mtrHandle->rotInfo->encPtr->Instance->CNT;
	  char buff[64];
	  sprintf(buff, "%ld\n",encoderVal);
	  printf(buff);

//    osDelay((int) PID.dt);
	  osDelay(100);
  }
  /* USER CODE END stepperCtrlFn */
}

/* USER CODE BEGIN Header_stateMachineFn */
/**
* @brief Function implementing the stateMachine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stateMachineFn */
void stateMachineFn(void *argument)
{
  /* USER CODE BEGIN stateMachineFn */
	// Set PWM duty cycle for buzzer to 50%
//	TIM3->CCR2 = (int) TIM3->ARR/2;
//	TIM3->CCR2 = 2000;

// Buzzer only sounds when state machine task entered initially
#ifdef USE_BUZZER
//	__HAL_TIM_SET_AUTORELOAD(PWMTimer, BUZZ_ARR-1);
//	__HAL_TIM_SET_PRESCALER(PWMTimer, BUZZ_PSC-1);
	__HAL_TIM_SET_COMPARE(PWMTimer, BUZZ_CHANNEL, ((int) PWMTimer->Instance->ARR)/2);

	HAL_TIM_PWM_Start(PWMTimer, BUZZ_CHANNEL);
	osDelay(100);
	HAL_TIM_PWM_Stop(PWMTimer, BUZZ_CHANNEL);
	osDelay(100);
	HAL_TIM_PWM_Start(PWMTimer, BUZZ_CHANNEL);
	osDelay(100);
	HAL_TIM_PWM_Stop(PWMTimer, BUZZ_CHANNEL);
	osDelay(100);
	HAL_TIM_PWM_Start(PWMTimer, BUZZ_CHANNEL);
	osDelay(100);
	HAL_TIM_PWM_Stop(PWMTimer, BUZZ_CHANNEL);

	osThreadFlagsSet(stepperCtrlTaskHandle, buzzerDone);
#endif

  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END stateMachineFn */
}

/* USER CODE BEGIN Header_decodeUSBFn */
/**
* @brief Function implementing the decodeUSBTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_decodeUSBFn */
void decodeUSBFn(void *argument)
{
  /* USER CODE BEGIN decodeUSBFn */

	/* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END decodeUSBFn */
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
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim == PWMStopTimer) {
		// Stop PWM from timer 1
		HAL_TIM_PWM_Stop(PWMTimer, TIM_CHANNEL_1);

		// FIMXE: Sleep the driver for now
//		HAL_GPIO_WritePin(MTR_NSLP_GPIO_Port, MTR_NSLP_Pin, GPIO_PIN_RESET);


  }

  if (htim == EncoderTimer) {
	  // Check if encoder was decreasing or increasing
	  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(EncoderTimer) == 1) {
		  numOfRevolutions -= 1;
	  }
	  else {
		  numOfRevolutions += 1;
	  }
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
	  // Flash error LED
	  HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
	  osDelay(500);

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
