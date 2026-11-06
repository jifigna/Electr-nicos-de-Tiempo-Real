/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MIV-RT - Hito 1, Parte E -- VERSION PARA STM32F411E-DISCO
  *
  * MIVRT_E_V2: misma logica que MIVRT_E (F407 + LIS3DSH por SPI), adaptada a la
  * placa 32F411EDISCOVERY:
  *   - CPU a 84 MHz (la F411 no admite 168 MHz)
  *   - base de tiempo de la HAL con TIM11 (la F411 no tiene TIM6)
  *   - acelerometro LSM303DLHC / LSM303AGR por I2C1 (PB6/PB9), no LIS3DSH por SPI
  * NO abrir el .ioc ni regenerar codigo: volveria a la configuracion del F407.
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { Verde, Naranja, Rojo, Fallo } estado_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---- Acelerometro LSM303DLHC / LSM303AGR (F411E-DISCO), I2C1 ----
#define ACC_ADDR        (0x19 << 1)  // direccion 7 bits 0x19 -> 8 bits 0x32
#define ACC_WHO_AM_I    0x0F         // AGR devuelve 0x33 (el DLHC puede no tenerlo)
#define ACC_CTRL_REG1   0x20
#define ACC_CTRL_REG4   0x23
#define ACC_OUT_X_L     0x28
#define ACC_AUTOINC     0x80         // en I2C, bit 7 de la subdireccion = autoincremento
#define ACC_MG_POR_LSB  0.0625f      // HR, +-2 g: 1 mg/digito en 12 bits alineados a la izquierda -> 1/16 mg por LSB de 16 bits
#define I2C_TIMEOUT_MS  5

// Ciclos de CPU por microsegundo (84 en esta placa). Sustituye al /168 de la F407.
#define CICLOS_US   (SystemCoreClock / 1000000U)

// ---- Reto 1.13: indice de vibracion ----
// W = muestras de Sensor que forman una ventana. Sensor muestrea cada 10 ms,
// asi que W = 10 -> ventana de 100 ms. Ajustar segun el retardo que pida RF-6.
#define W           10
#define VON         30.0f  // mg  (ajustar con las medidas del Reto 1.13.2)
#define VOFF        20.0f  // mg  (VOFF < VON -> histeresis)

// ---- Conmutadores de experimentos previos (Paso E.2: TODOS a 0) ----
#define EJEMPLO_SOBRECARGA  0   // Reto 1.9: Proceso con C = 12 ms
#define EJEMPLO_CARRERA     0   // Reto 1.12: tarea Carrera + incrementa()
#define EJEMPLO_FPU         0   // Reto 1.10: comparacion float/double
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
/* Definitions for Carrera */
osThreadId_t CarreraHandle;
const osThreadAttr_t Carrera_attributes = {
  .name = "Carrera",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
// ---- Sensor / arranque ----
volatile uint8_t who = 0;
volatile uint8_t ctrl1 = 0, ctrl4 = 0;     // relectura de configuracion (0x57 y 0x88)
volatile uint8_t acc_ack = 0;              // 1 si el acelerometro respondio en I2C
volatile uint32_t i2c_errores = 0;         // lecturas I2C fallidas en Sensor
volatile uint8_t init_err = 1;
volatile uint8_t raw[6];
volatile int16_t ax, ay, az;          // ultima muestra publicada por Sensor
volatile uint32_t seq_sensor = 0;     // numero de secuencia de esa muestra (1.13.3)

// ---- Copia coherente que usa Proceso ----
volatile float ax_mg, ay_mg, az_mg;
volatile uint32_t seq_proceso = 0;    // secuencia de la muestra que esta usando Proceso

// ---- Tiempos de computo (us) ----
volatile uint32_t c_sensor_max_us = 0;
volatile uint32_t c_proceso_max_us = 0;
volatile uint32_t c_seguridad_max_us = 0;
volatile uint32_t c_comms_max_us = 0;

// ---- Reto 1.8: periodo y jitter de Sensor ----
volatile uint32_t sensor_periodo_us = 0;
volatile uint32_t sensor_per_max_us = 0;
volatile uint32_t sensor_per_min_us = 999999;
volatile uint32_t sensor_jitter_us = 0;

// ---- Reto 1.9: tiempo de respuesta ----
volatile uint32_t sensor_R_ms = 0;
volatile uint32_t sensor_R_max_ms = 0;
volatile uint32_t sensor_perdidos = 0;

// ---- Retos 1.10 / 1.11 ----
volatile float pitch_grados, roll_grados;
volatile float inclinacion = 0.0f;
volatile estado_t estado = Verde;
#if EJEMPLO_FPU
volatile uint32_t t_float, t_double;
#endif

// ---- Reto 1.12 ----
volatile uint32_t contador = 0;
volatile uint32_t privado_sensor = 0, privado_carrera = 0;

// ---- Reto 1.13: vibracion ----
volatile float a_mod = 0.0f;          // |a| de la ultima muestra usada (mg)
volatile float V = 0.0f;              // indice de vibracion de la ultima ventana cerrada (mg)
volatile float V_max = 0.0f;          // maximo V visto (para caracterizar; poner a 0 desde el debugger)
volatile uint8_t vibracion = 0;       // estado del LED azul
static float ventana[W];              // |a| de las muestras de la ventana actual que llegan a Proceso
static uint32_t n_ventana = 0;        // cuantas muestras van en la ventana actual
static uint32_t id_ventana = 0;       // numero de ventana actual (= (seq-1)/W)
volatile uint32_t ventanas_cerradas = 0;
volatile uint32_t usadas_ultima_ventana = 0;    // de cada W muestras de Sensor, cuantas usa Proceso
volatile uint32_t perdidas_ultima_ventana = 0;  // W - usadas
volatile uint32_t muestras_usadas = 0;          // totales
volatile uint32_t muestras_perdidas = 0;        // totales (saltos de secuencia)
volatile uint32_t muestras_repetidas = 0;       // Proceso vio la misma secuencia dos veces
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
void Task_carrera(void *argument);

/* USER CODE BEGIN PFP */
__attribute__((unused)) static void MX_I2S3_Init(void);
__attribute__((unused)) static void MX_SPI1_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ---------------- Driver LSM303 (acelerometro) por I2C1 ----------------
static HAL_StatusTypeDef acc_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef acc_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
    if (n > 1) reg |= ACC_AUTOINC;
    return HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, n, I2C_TIMEOUT_MS);
}

// Devuelve 0 si el sensor responde y queda configurado; 1 si no (RF-4).
uint8_t acc_init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, ACC_ADDR, 3, I2C_TIMEOUT_MS) != HAL_OK) return 1;
    acc_ack = 1;

    if (acc_write(ACC_CTRL_REG1, 0x57) != HAL_OK) return 1;   // ODR 100 Hz, modo normal, X/Y/Z
    if (acc_write(ACC_CTRL_REG4, 0x88) != HAL_OK) return 1;   // BDU = 1, +-2 g, alta resolucion
    return 0;
}

// Sensor: lee y publica la muestra + su numero de secuencia de forma atomica
static void leer_sensor(void)
{
    uint8_t b[6];
    if (acc_read(ACC_OUT_X_L, b, 6) != HAL_OK) {   // la transaccion I2C va FUERA de la seccion critica
        i2c_errores++;
        return;                                    // sin muestra nueva: no se publica ni avanza la secuencia
    }

    taskENTER_CRITICAL();
    for (int i = 0; i < 6; i++) raw[i] = b[i];
    ax = (int16_t)((b[1] << 8) | b[0]);
    ay = (int16_t)((b[3] << 8) | b[2]);
    az = (int16_t)((b[5] << 8) | b[4]);
    seq_sensor++;                               // (1.13.3) numero de secuencia
    taskEXIT_CRITICAL();
}

// Proceso: copia coherente de los tres ejes + su secuencia (Reto 1.12.3)
static void convertir_mg(void)
{
    taskENTER_CRITICAL();
    int16_t x = ax, y = ay, z = az;
    uint32_t s = seq_sensor;
    taskEXIT_CRITICAL();

    ax_mg = x * ACC_MG_POR_LSB;
    ay_mg = y * ACC_MG_POR_LSB;
    az_mg = z * ACC_MG_POR_LSB;
    seq_proceso = s;
}

static void calcular_angulos(void)
{
    pitch_grados = atan2f(ax_mg, sqrtf(ay_mg*ay_mg + az_mg*az_mg)) * 57.29578f;
    roll_grados  = atan2f(ay_mg, az_mg) * 57.29578f;
}

#if EJEMPLO_FPU
static void comparar_fpu(void)
{
    uint32_t t0 = DWT->CYCCNT;
    volatile float pf = atan2f(ax_mg, sqrtf(ay_mg*ay_mg + az_mg*az_mg)) * 57.29578f;
    t_float = DWT->CYCCNT - t0;

    t0 = DWT->CYCCNT;
    volatile double pd = atan2((double)ax_mg, sqrt((double)ay_mg*ay_mg + (double)az_mg*az_mg)) * 57.29578;
    t_double = DWT->CYCCNT - t0;
    (void)pf; (void)pd;
}
#endif

// Desviacion tipica de las n muestras de la ventana
static float desviacion(const float *v, uint32_t n)
{
    float media = 0.0f;
    for (uint32_t i = 0; i < n; i++) media += v[i];
    media /= (float)n;

    float suma = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float e = v[i] - media;
        suma += e * e;
    }
    return sqrtf(suma / (float)n);
}

// Reto 1.13: V = desviacion tipica de |a| en ventanas de W muestras de Sensor.
// Proceso (20 ms) solo ve 1 de cada 2 muestras de Sensor (10 ms): las cuenta.
static void calcular_vibracion(void)
{
    static uint32_t seq_ant = 0;
    uint32_t s = seq_proceso;

    if (s == 0) return;                         // Sensor aun no ha publicado nada

    if (s == seq_ant) {                         // misma muestra que la vez anterior
        muestras_repetidas++;
        return;
    }
    if (seq_ant != 0 && (s - seq_ant) > 1)
        muestras_perdidas += (s - seq_ant - 1);
    seq_ant = s;
    muestras_usadas++;

    // A que ventana pertenece esta muestra: seq 1..W -> 0, W+1..2W -> 1, ...
    uint32_t id = (s - 1) / W;

    if (id != id_ventana) {                     // se cierra la ventana anterior
        if (n_ventana >= 2) {
            V = desviacion(ventana, n_ventana);
            if (V > V_max) V_max = V;

            // Histeresis (LED azul)
            if (!vibracion && V >= VON)       vibracion = 1;
            else if (vibracion && V <= VOFF)  vibracion = 0;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, vibracion ? GPIO_PIN_SET : GPIO_PIN_RESET);

            usadas_ultima_ventana   = n_ventana;
            perdidas_ultima_ventana = W - n_ventana;
            ventanas_cerradas++;
        }
        id_ventana = id;
        n_ventana = 0;
    }

    a_mod = sqrtf(ax_mg*ax_mg + ay_mg*ay_mg + az_mg*az_mg);
    if (n_ventana < W) ventana[n_ventana++] = a_mod;
}

static void maquina_estados(void)
{
    inclinacion = fmaxf(fabsf(pitch_grados), fabsf(roll_grados));

    switch (estado) {
        case Verde:
            if (inclinacion >= 30.0f)      estado = Rojo;
            else if (inclinacion >= 15.0f) estado = Naranja;
            break;
        case Naranja:
            if (inclinacion >= 30.0f)      estado = Rojo;
            else if (inclinacion < 13.0f)  estado = Verde;
            break;
        case Rojo:
            if (inclinacion < 13.0f)       estado = Verde;
            else if (inclinacion < 28.0f)  estado = Naranja;
            break;
        case Fallo:
            break;                          // RF-4: de FALLO no se sale
    }

    switch (estado) {
        case Verde:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
            break;
        case Naranja:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
            break;
        case Rojo:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
            break;
        case Fallo:
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);   // rojo parpadeando
            break;
    }
}

#if EJEMPLO_SOBRECARGA || EJEMPLO_CARRERA
void quemar_cpu_us(uint32_t us)
{
    uint32_t acc = 0, prev = DWT->CYCCNT;
    while (acc < us * CICLOS_US) {
        uint32_t now = DWT->CYCCNT, d = now - prev; prev = now;
        if (d < 10u * CICLOS_US) acc += d;
    }
}
#endif

#if EJEMPLO_CARRERA
void incrementa(void)
{
    taskENTER_CRITICAL();
    uint32_t tmp = contador;
    quemar_cpu_us(200);
    contador = tmp + 1;
    taskEXIT_CRITICAL();
}
#endif

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
  MX_I2C1_Init();      // acelerometro LSM303 (en la F407 era el codec de audio)
  //MX_I2S3_Init();    // audio: no se usa
  //MX_SPI1_Init();    // en la F411 el SPI1 es del giroscopio: no se usa
  /* USER CODE BEGIN 2 */
  // Contador de ciclos DWT (medidas de C, periodo y jitter)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_Delay(10);                                // arranque del sensor

  init_err = acc_init();
  if (init_err != 0) estado = Fallo;            // RF-4

  acc_read(ACC_WHO_AM_I,  (uint8_t*)&who,   1); // LSM303AGR: 0x33 (DLHC: puede dar otro valor, no importa)
  acc_read(ACC_CTRL_REG1, (uint8_t*)&ctrl1, 1); // debe dar 0x57
  acc_read(ACC_CTRL_REG4, (uint8_t*)&ctrl4, 1); // debe dar 0x88
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

  /* creation of Carrera */
  CarreraHandle = osThreadNew(Task_carrera, NULL, &Carrera_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  //osThreadSetPriority(ProcesoHandle, osPriorityRealtime);   // Reto 1.9 caso (b) descomentar para poner Proceso por encima de Sensor
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
  // F411: VOS = 11 (escala 1). Se escribe a mano porque la macro de la HAL del F407 solo toca 1 bit.
  PWR->CR = (PWR->CR & ~(3UL << 14)) | (3UL << 14);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;   // 8/8*336/4 = 84 MHz (maximo F411: 100 MHz)
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // 42 MHz (max 50 MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // 84 MHz (max 100 MHz)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)   // F411 a 3,3 V y 84 MHz: 2 WS
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
  hi2c1.Init.ClockSpeed = 400000;     // I2C rapido: lectura de 6 bytes en ~0,2 ms
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
  * @brief  Sensor: T = 10 ms, prioridad High (RM).
  */
/* USER CODE END Header_Task_sensor */
void Task_sensor(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t t_ant = 0;
  uint32_t n = 0;
  uint32_t t_sig = osKernelGetTickCount();

  for(;;)
  {
    uint32_t t_act = t_sig;

    // Reto 1.8: periodo y jitter
    uint32_t ahora = DWT->CYCCNT;
    if (n >= 2) {
        sensor_periodo_us = (ahora - t_ant) / CICLOS_US;
        if (sensor_periodo_us > sensor_per_max_us) sensor_per_max_us = sensor_periodo_us;
        if (sensor_periodo_us < sensor_per_min_us) sensor_per_min_us = sensor_periodo_us;
        sensor_jitter_us = sensor_per_max_us - sensor_per_min_us;
    }
    t_ant = ahora;
    n++;

    uint32_t t0 = DWT->CYCCNT;
    leer_sensor();
#if EJEMPLO_CARRERA
    incrementa();
    privado_sensor++;
#endif
    uint32_t c = (DWT->CYCCNT - t0) / CICLOS_US;
    if (c > c_sensor_max_us) c_sensor_max_us = c;

    // Reto 1.9: tiempo de respuesta
    sensor_R_ms = osKernelGetTickCount() - t_act;
    if (sensor_R_ms > sensor_R_max_ms) sensor_R_max_ms = sensor_R_ms;
    if (sensor_R_ms > 10) sensor_perdidos++;

    t_sig += 10;
    osDelayUntil(t_sig);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_proceso */
/**
* @brief Proceso: T = 20 ms, prioridad AboveNormal.
*/
/* USER CODE END Header_Task_proceso */
void Task_proceso(void *argument)
{
  /* USER CODE BEGIN Task_proceso */
  uint32_t t_sig = osKernelGetTickCount();
  for(;;)
  {
    uint32_t t0 = DWT->CYCCNT;

    convertir_mg();
    calcular_angulos();
    calcular_vibracion();
#if EJEMPLO_FPU
    comparar_fpu();
#endif
#if EJEMPLO_SOBRECARGA
    quemar_cpu_us(12000);             // Reto 1.9
#endif

    uint32_t c = (DWT->CYCCNT - t0) / CICLOS_US;
    if (c > c_proceso_max_us) c_proceso_max_us = c;

    t_sig += 20;
    osDelayUntil(t_sig);
  }
  /* USER CODE END Task_proceso */
}

/* USER CODE BEGIN Header_Task_seguridad */
/**
* @brief Seguridad: T = 50 ms, prioridad Normal.
*/
/* USER CODE END Header_Task_seguridad */
void Task_seguridad(void *argument)
{
  /* USER CODE BEGIN Task_seguridad */
  uint32_t t_sig = osKernelGetTickCount();
  for(;;)
  {
    uint32_t t0 = DWT->CYCCNT;
    maquina_estados();
    uint32_t c = (DWT->CYCCNT - t0) / CICLOS_US;
    if (c > c_seguridad_max_us) c_seguridad_max_us = c;

    t_sig += 50;
    osDelayUntil(t_sig);
  }
  /* USER CODE END Task_seguridad */
}

/* USER CODE BEGIN Header_Task_comms */
/**
* @brief Comms: T = 100 ms, prioridad BelowNormal. Creada pero vacia en el Hito 1.
*/
/* USER CODE END Header_Task_comms */
void Task_comms(void *argument)
{
  /* USER CODE BEGIN Task_comms */
  uint32_t t_sig = osKernelGetTickCount();
  for(;;)
  {
    uint32_t t0 = DWT->CYCCNT;
    /* vacia en el Hito 1 */
    uint32_t c = (DWT->CYCCNT - t0) / CICLOS_US;
    if (c > c_comms_max_us) c_comms_max_us = c;

    t_sig += 100;
    osDelayUntil(t_sig);
  }
  /* USER CODE END Task_comms */
}

/* USER CODE BEGIN Header_Task_carrera */
/**
* @brief Carrera (Reto 1.12). Con EJEMPLO_CARRERA = 0 termina nada mas arrancar.
*/
/* USER CODE END Header_Task_carrera */
void Task_carrera(void *argument)
{
  /* USER CODE BEGIN Task_carrera */
#if EJEMPLO_CARRERA
  for(;;)
  {
    incrementa();
    privado_carrera++;
  }
#else
  osThreadExit();                     // ejemplo desactivado (Paso E.2)
#endif
  /* USER CODE END Task_carrera */
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
  if (htim->Instance == TIM11)
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
