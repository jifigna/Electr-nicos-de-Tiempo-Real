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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIS_READ 0x80 // bit de lectura en el primer byte
#define WHOAMI_REG 0x0F
#define WHOAMI_VAL 0x3F // firma del LIS3DSH
#define LIS_READ    0x80
#define WHOAMI_REG  0x0F
#define CTRL_REG4   0x20
#define CTRL_REG5   0x24
#define CTRL_REG6   0x25
#define OUT_X_L     0x28
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* Definitions for Sensor */
osThreadId_t SensorHandle;
const osThreadAttr_t Sensor_attributes = {
  .name = "Sensor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Proceso */
osThreadId_t ProcesoHandle;
const osThreadAttr_t Proceso_attributes = {
  .name = "Proceso",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Seguridad */
osThreadId_t SeguridadHandle;
const osThreadAttr_t Seguridad_attributes = {
  .name = "Seguridad",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for comms */
osThreadId_t commsHandle;
const osThreadAttr_t comms_attributes = {
  .name = "comms",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */
volatile uint8_t who = 0;
volatile uint8_t r4 = 0;
volatile uint8_t r5 = 0;
volatile uint8_t r6 = 0;
volatile uint8_t init_err = 1;
volatile uint8_t raw[6];
volatile int16_t ax, ay, az;
volatile float ax_mg, ay_mg, az_mg;
volatile uint32_t c_sensor_max_us=0;
volatile uint32_t c_proceso_max_us=0;
volatile uint32_t c_seguridad_max_us=0;
volatile uint32_t c_comms_max_us=0;

volatile uint32_t sensor_periodo_us = 0;
volatile uint32_t sensor_per_max_us = 0;
volatile uint32_t sensor_per_min_us = 999999;
volatile uint32_t sensor_jitter_us = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void Task_sensor(void *argument);
void Task_proceso(void *argument);
void Task_seguridad(void *argument);
void Task_comms(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void cs_low(void)  { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); }

// Lee 'n' (<= 6) registros consecutivos a partir de 'reg'.
void lis_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
    uint8_t tx[7] = {0}, rx[7];
    tx[0] = LIS_READ | reg;                     // dirección + bit de lectura

    cs_low();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, n + 1, HAL_MAX_DELAY);  // una transacción de n+1 bytes
    cs_high();

    for (uint16_t i = 0; i < n; i++)
        buf[i] = rx[i + 1];                     // rx[0] se descarta (el sensor aún no contestaba)
}

void lis_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;                         // dirección con bit 7 = 0 (escritura)
    tx[1] = val;                                // valor a escribir

    cs_low();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);  // solo envío
    cs_high();
}


//PARTE B//

// Devuelve 0 si el sensor responde y queda configurado; 1 si no.
uint8_t lis_init(void)
{
    uint8_t id;
    lis_read(WHOAMI_REG, &id, 1);               // prueba de vida
    if (id != WHOAMI_VAL) return 1;             // si no es 0x3F → error

    lis_write(CTRL_REG4, 0x6F);                 // ODR 100 Hz, BDU = 1, X/Y/Z
    lis_write(CTRL_REG5, 0xC0);                 // ← faltaba: BW 50 Hz, ±2 g, sin self-test, 4 hilos
    lis_write(CTRL_REG6, 0x10);                 // ADD_INC = 1
    return 0;
}

static void leer_sensor(void)     // cada 10 ms
{
    lis_read(OUT_X_L, (uint8_t*)raw, 6);
    ax = (int16_t)((raw[1] << 8) | raw[0]);
    ay = (int16_t)((raw[3] << 8) | raw[2]);
    az = (int16_t)((raw[5] << 8) | raw[4]);
}

static void convertir_mg(void)    // cada 20 ms
{
    ax_mg = ax * 0.06f;   //f para que l multiplicación se haga con 32 bits (en vez de double)
    ay_mg = ay * 0.06f;
    az_mg = az * 0.06f;
}

static void latido(void)          // cada 50 ms
{
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);   // LED verde
}

static void actividad_larga(void)     // tarea de 15 ms, CPU para sin hacer nada
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 15) { }
}


void quemar_cpu_us(uint32_t us)
{
    uint32_t acc = 0, prev = DWT->CYCCNT;
    while (acc < us * 168u) {
        uint32_t now = DWT->CYCCNT, d = now - prev; prev = now;
        if (d < 1680u) acc += d;
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

   // Lee 'n' (<= 6) registros consecutivos a partir de 'reg'.
  //__HAL_SPI_ENABLE(&hspi1);                     // enciende el SPI1 ya: SCK queda en reposo alto antes de bajar CS
  //HAL_Delay(10);                                // da tiempo al sensor a arrancar
  cs_high();                                    // CS en reposo
  init_err = lis_init();                        // 0 = OK
  lis_read(WHOAMI_REG, (uint8_t*)&who, 1);      // debe dar 0x3F
  lis_read(CTRL_REG4,  (uint8_t*)&r4, 1);       // debe dar 0x6F
  lis_read(CTRL_REG5,  (uint8_t*)&r5, 1);       // debe dar 0xC0
  lis_read(CTRL_REG6,  (uint8_t*)&r6, 1);       // debe dar 0x10



  ////////PARTE B///////

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
  /* creation of Sensor */
  SensorHandle = osThreadNew(Task_sensor, NULL, &Sensor_attributes);

  /* creation of Proceso */
  ProcesoHandle = osThreadNew(Task_proceso, NULL, &Proceso_attributes);

  /* creation of Seguridad */
  SeguridadHandle = osThreadNew(Task_seguridad, NULL, &Seguridad_attributes);

  /* creation of comms */
  commsHandle = osThreadNew(Task_comms, NULL, &comms_attributes);

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

  /* USER CODE END 3 */
}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_sensor */
/**
  * @brief  Function implementing the Sensor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_sensor */
void Task_sensor(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	  uint32_t t_ant = 0;
	  uint32_t n = 0;
	  uint32_t t_sig = osKernelGetTickCount();

	  for(;;)
	  {

	    uint32_t ahora = DWT->CYCCNT;
	    if (n >= 2) {
	        sensor_periodo_us = (ahora - t_ant) / 168;
	        if (sensor_periodo_us > sensor_per_max_us) sensor_per_max_us = sensor_periodo_us;
	        if (sensor_periodo_us < sensor_per_min_us) sensor_per_min_us = sensor_periodo_us;
	        sensor_jitter_us = sensor_per_max_us - sensor_per_min_us;
	    }
	    t_ant = ahora;
	    n++;

	    // lo que ya tenías
	    uint32_t t0 = DWT->CYCCNT;
	    leer_sensor();
	    quemar_cpu_us(2000);                                // carga artificial de 2 ms
	    uint32_t c = (DWT->CYCCNT - t0) / 168;
	    if (c > c_sensor_max_us) c_sensor_max_us = c;

	    //osDelay(10);
	    t_sig+=10;
	    osDelayUntil(t_sig);
	  }
	  /* USER CODE END 5 */
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_proceso */
/**
* @brief Function implementing the Proceso thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_proceso */
void Task_proceso(void *argument)
{
  /* USER CODE BEGIN Task_proceso */
  /* Infinite loop */
  for(;;)
  {
	  uint32_t t0=DWT->CYCCNT;
	  convertir_mg();
	  uint32_t c=(DWT->CYCCNT-t0)/168;

	  if (c>c_proceso_max_us) c_proceso_max_us=c;
      osDelay(10);
  }
  /* USER CODE END Task_proceso */
}

/* USER CODE BEGIN Header_Task_seguridad */
/**
* @brief Function implementing the Seguridad thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_seguridad */
void Task_seguridad(void *argument)
{
  /* USER CODE BEGIN Task_seguridad */
  /* Infinite loop */
  for(;;)
  {
	  uint32_t t0=DWT->CYCCNT;
	  latido();
	  uint32_t c=(DWT->CYCCNT-t0)/168;

	  if (c>c_seguridad_max_us) c_seguridad_max_us=c;
	  osDelay(10);

  }
  /* USER CODE END Task_seguridad */
}

/* USER CODE BEGIN Header_Task_comms */
/**
* @brief Function implementing the comms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_comms */
void Task_comms(void *argument)
{
  /* USER CODE BEGIN Task_comms */
  /* Infinite loop */
  for(;;)
  {

	  uint32_t t0=DWT->CYCCNT;

	  uint32_t c=(DWT->CYCCNT-t0)/168;

	  if (c>c_comms_max_us) c_comms_max_us=c;
	   osDelay(10);
  }
  /* USER CODE END Task_comms */
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
