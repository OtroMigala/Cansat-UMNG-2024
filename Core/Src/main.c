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
#include <bmp280.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <uart.h>
#include <mpu9250.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_BUFFER_SIZE 1024
uint8_t gpsBuffer[GPS_BUFFER_SIZE];
volatile uint32_t gpsBufferIndex = 0;
volatile uint8_t gpsDataReady = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;
float pressure;
float temperature;
float humidity;
uint16_t size;
uint8_t Data[256];
//MODULO GPS
uint8_t Rxdata[750];
char Txdata[750];
uint8_t Flag=0;
char *ptr;

char GPS_Payload[100];
float time = 0, latitude = 0, longitude = 0;
char lat_dir = 'N', long_dir = 'E';
int hours = 0, minutes = 0;
float seconds = 0;

volatile uint32_t lastTransmissionTime = 0;
	//variables para transmitir

uint16_t tamaño=0;
char Mensaje_a_enviar_var_unidas[500];
char mensaje[500];

//altitud:

float gps_altitude = 0; // Nueva variable para la altitud del GPS

//Press=82893.48Pa
#define FILTER_SIZE 10
#define PO 74450.0f //Presion en cajicá
//#define PO 82318.98f  // Presión en lab de camilo
#define RO 1.225f    // Densidad del aire a nivel del mar en kg/m³
#define G 9.81f      // Aceleración debido a la gravedad en m/s²

float pressureBuffer[FILTER_SIZE];
int bufferIndex = 0;

///ahorrodeenergia
uint32_t lastWakeTime = 0;
const uint32_t SLEEP_INTERVAL = 5000; // 5 segundos de intervalo de sueño
const uint32_t TRANSMIT_INTERVAL = 60000; // 1 minuto entre transmisiones
//servo

#define SERVO_MIN_PULSE 1000   // 0 grados
#define SERVO_MAX_PULSE 2000  // 180 grados
#define SERVO_FILTER_SIZE 5
float servoAngleBuffer[SERVO_FILTER_SIZE] = {0};
int servoBufferIndex = 0;

#define ALTITUDE_MIN 0        // Altitud mínima en metros
#define ALTITUDE_TARGET 200.0f   // Altitud objetivo en metros (1.5m como especificaste)
#define ALTITUDE_MAX 500       // Altitud máxima en metros

TIM_HandleTypeDef htim3;
float baseAltitude = 0;      // Altitud base (nivel del suelo)

float lastAltitude = 0;
bool passedFirstPeak = false;
float servoAngle = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void process_gps_data(void);
void send_gps_data(void);
void Format_data(float time, float lat, char lat_dir, float lon, char long_dir, float gps_alt);
void LoRa_Init(void);
void initializeServo(void);
void LoRa_Send(const char* data);
float movingAverageFilter(float newValue);
float calculateAltitude(float pressure);
void moveServoBasedOnAltitude(float altitude);
float filterServoAngle(float newAngle);
void moveServoToAngle(float angle);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  MX_TIM3_Init();
  HAL_TIM_Base_Start(&htim3);
  initializeServo();

  char debug[100];
  snprintf(debug, sizeof(debug), "Timer3 config: PSC=%lu, ARR=%lu, Frecuencia=%lu Hz\r\n",
           htim3.Init.Prescaler, htim3.Init.Period,
           HAL_RCC_GetPCLK1Freq() / ((htim3.Init.Prescaler + 1) * (htim3.Init.Period + 1)));
  HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), HAL_MAX_DELAY);

  LoRa_Init();
  HAL_UART_Receive_DMA(&huart2, (uint8_t*)gpsBuffer, GPS_BUFFER_SIZE);
  lastTransmissionTime = 0 ;

  //En estas lineas pasan cosas raras, aquí es donde se inicializa el dispositivo
  	  	bmp280_init_default_params(&bmp280.params);
    	bmp280.addr = BMP280_I2C_ADDRESS_0;
    	bmp280.i2c = &hi2c1;


    	while (!bmp280_init(&bmp280, &bmp280.params)) {
    	    sprintf((char *)Data, "BMP280 initialization failed\n");
    	    //uartx_write_text(&huart1, Data);
    	    HAL_Delay(1000);
    	}
	bool bme280p = bmp280.id == BMP280_CHIP_ID;
	sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
	uartx_write_text(&huart1, Data);



	 float temp, pressure, humidity;
	 bmp280_read_float(&bmp280, &temp, &pressure, &humidity);

	 for (int i = 0; i < 20; i++) {  // Promedio de 10 lecturas
	     float temp, pressure, humidity;
	     bmp280_read_float(&bmp280, &temp, &pressure, &humidity);
	     baseAltitude += calculateAltitude(pressure);
	     HAL_Delay(100);  // Pequeña pausa entre lecturas
	 }
	 baseAltitude /= 20.0f;

	 //gps

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t currentTime = HAL_GetTick();

	     if (currentTime - lastTransmissionTime >= 1000)  // 1000 ms = 1 segundo
	     {
	         // Leer datos del GPS
	         process_gps_data();

	         // Leer datos del BMP280
	         float temperature, pressure;
	         bmp280_read_float(&bmp280, &temperature, &pressure, NULL);

	         // Aplicar filtro de media móvil a la presión
	         float filteredPressure = movingAverageFilter(pressure);

	         // Calcular altitud con el BMP280
	         float bmp280_altitude = calculateAltitude(filteredPressure);
	         float relativeAltitude = bmp280_altitude - baseAltitude;

	         // Solo mover el servo si es necesario
	         moveServoBasedOnAltitude(bmp280_altitude);

	         // Leer datos del MPU9250
	         float accelData[3], gyroData[3], magData[3];
	         MPU9250_ReadAccel(&hi2c1, accelData);
	         MPU9250_ReadGyro(&hi2c1, gyroData);
	         MPU9250_ReadMag(&hi2c1, magData);

	         // Formar el mensaje con los datos
	         char buffer[512];
	         int len = snprintf(buffer, sizeof(buffer),
	             "Time=%02d:%02d:%05.2f,Lat=%.6f,Long=%.6f,"
	             "Temp=%.2fC,Press=%.2fPa,RelAlt=%.2fm,"
	             "AccX=%.2f,AccY=%.2f,AccZ=%.2f,"
	             "GyroX=%.2f,GyroY=%.2f,GyroZ=%.2f,"
	             "MagX=%.2f,MagY=%.2f,MagZ=%.2f\r\n",
	             hours, minutes, seconds, latitude, longitude,
	             temperature, filteredPressure, relativeAltitude,
	             accelData[0], accelData[1], accelData[2],
	             gyroData[0], gyroData[1], gyroData[2],
	             magData[0], magData[1], magData[2]);

	         // Enviar el mensaje por UART1
	         HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);

	         // Enviar el mensaje por LoRa
	         LoRa_Send(buffer);

	         lastTransmissionTime = currentTime;
	     }

	     // Entrar en modo de bajo consumo
	     HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hi2c1.Init.ClockSpeed = 115200;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void process_gps_data(void)
{
    if (gpsDataReady)
    {
        char *ptr_rmc = strstr((char*)gpsBuffer, "$GPRMC");
        char *ptr_gga = strstr((char*)gpsBuffer, "$GPGGA");

        if (ptr_rmc != NULL)
        {
            if (sscanf(ptr_rmc, "$GPRMC,%f,A,%f,%c,%f,%c", &time, &latitude, &lat_dir, &longitude, &long_dir) == 5)
            {
                // Convertir latitud y longitud a grados decimales
                float lat_degrees = (int)(latitude / 100);
                float lat_minutes = latitude - (lat_degrees * 100);
                latitude = lat_degrees + (lat_minutes / 60);
                if (lat_dir == 'S') latitude = -latitude;

                float long_degrees = (int)(longitude / 100);
                float long_minutes = longitude - (long_degrees * 100);
                longitude = long_degrees + (long_minutes / 60);
                if (long_dir == 'W') longitude = -longitude;

                // Convertir tiempo
                hours = (int)time / 10000;
                minutes = (int)(time - (hours * 10000)) / 100;
                seconds = time - ((int)time / 100) * 100;
            }
        }

        if (ptr_gga != NULL)
        {
            if (sscanf(ptr_gga, "$GPGGA,%*f,%*f,%*c,%*f,%*c,%*d,%*d,%*f,%f", &gps_altitude) == 1)
            {
                // La altitud del GPS se ha extraído correctamente
            }
            else
            {
                gps_altitude = 0; // Si no se pudo extraer, se establece a 0
            }
        }

        if (ptr_rmc == NULL && ptr_gga == NULL)
        {
            // Si no se encontró ninguna de las dos cadenas, invalidar los datos
            time = 0;
            latitude = 0;
            longitude = 0;
            gps_altitude = 0;
        }

        gpsDataReady = 0;
        gpsBufferIndex = 0;
        HAL_UART_Receive_DMA(&huart2, gpsBuffer, GPS_BUFFER_SIZE);
    }
}

void send_gps_data(void)
{
    if (time != 0 && latitude != 0 && longitude != 0)
    {
        Format_data(time, latitude, lat_dir, longitude, long_dir, gps_altitude);
    }
}

void Format_data(float time, float lat, char lat_dir, float lon, char long_dir, float gps_alt)
{
    char Data[150];
    snprintf(Data, sizeof(Data), "\r\nTime=%02d:%02d:%05.2f, Lat=%.6f, Long=%.6f, GPS Alt=%.2f m\r\n",
             hours, minutes, seconds, lat, lon, gps_alt);
    HAL_UART_Transmit(&huart1, (uint8_t*)Data, strlen(Data), 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        gpsDataReady = 1;
    }
}

//funciones del Lora:

/* USER CODE BEGIN 4 */
void LoRa_Init(void)
{

    // Configurar el módulo LoRa
    HAL_UART_Transmit(&huart3, (uint8_t*)"AT+MODE=0\r\n", 11, 1000);
    HAL_Delay(100);

    // Configurar la frecuencia (ajusta según tu región)
    HAL_UART_Transmit(&huart3, (uint8_t*)"AT+BAND=868500000\r\n", 20, 1000);
    HAL_Delay(100);

    // Configurar los parámetros de transmisión
    HAL_UART_Transmit(&huart3, (uint8_t*)"AT+PARAMETER=10,7,1,7\r\n", 24, 1000);
    HAL_Delay(100);

    // Configurar la potencia de transmisión
    // A mayor potencia de transmisión, mayor consumo de corriente (a tener en cuenta)
    HAL_UART_Transmit(&huart3, (uint8_t*)"AT+CRFOP=6\r\n", 13, 1000);
    HAL_Delay(100);
}

void LoRa_Send(const char* data)
{
    char cmd[600];
    int len = snprintf(cmd, sizeof(cmd), "AT+SEND=0,%d,%s\r\n", strlen(data), data);
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, len, HAL_MAX_DELAY);
}
float movingAverageFilter(float newValue) {
    static int count = 0;
    static float sum = 0;

    if (count < FILTER_SIZE) {
        pressureBuffer[count] = newValue;
        sum += newValue;
        count++;
    } else {
        sum -= pressureBuffer[bufferIndex];
        pressureBuffer[bufferIndex] = newValue;
        sum += newValue;
        bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    }

    return sum / count;
}
float filterServoAngle(float newAngle) {
    static int count = 0;
    static float sum = 0;

    if (count < SERVO_FILTER_SIZE) {
        servoAngleBuffer[count] = newAngle;
        sum += newAngle;
        count++;
    } else {
        sum -= servoAngleBuffer[servoBufferIndex];
        servoAngleBuffer[servoBufferIndex] = newAngle;
        sum += newAngle;
        servoBufferIndex = (servoBufferIndex + 1) % SERVO_FILTER_SIZE;
    }

    return sum / count;
}

void moveServoToAngle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t pulse = SERVO_MIN_PULSE + (uint32_t)((angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);

    char debug[100];
    snprintf(debug, sizeof(debug), "Servo movido a %.1f grados, pulse: %lu\r\n", angle, pulse);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), HAL_MAX_DELAY);

    HAL_Delay(20);  // Pequeña pausa para estabilizar
}
void moveServoBasedOnAltitude(float altitude) {
    static float lastAltitude = 0;
    static float maxAltitude = 0;
    static bool parachuteDeployed = false;
    static uint32_t lastMoveTime = 0;
    static const float DEPLOY_ALTITUDE = 200.0f; // Altura de despliegue en metros
    static const float ALTITUDE_HYSTERESIS = 2.0f; // Aumentamos la histéresis para mayor altura
    static const uint32_t MOVE_DELAY = 1000; // Retraso mínimo entre comprobaciones en ms

    float relativeAltitude = altitude - baseAltitude;
    uint32_t currentTime = HAL_GetTick();

    // Actualizar la altitud máxima alcanzada
    if (relativeAltitude > maxAltitude) {
        maxAltitude = relativeAltitude;
    }

    // Verificar si ha pasado suficiente tiempo desde la última comprobación
    if (currentTime - lastMoveTime < MOVE_DELAY) {
        return;
    }

    // Comprobar si hemos alcanzado o superado la altura de despliegue
    if (!parachuteDeployed && relativeAltitude >= DEPLOY_ALTITUDE) {
        // Activar el servo para desplegar el paracaídas
        moveServoToAngle(60.0f);  // Ajustamos a 90 grados para un despliegue completo

        parachuteDeployed = true;

        // Imprimir información de depuración
        char debug[100];
        snprintf(debug, sizeof(debug), "Paracaídas desplegado! Alt: %.2f m, Max Alt: %.2f m, Servo: 90.0 deg\r\n",
                 relativeAltitude, maxAltitude);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), HAL_MAX_DELAY);
    }

    lastAltitude = relativeAltitude;
    lastMoveTime = currentTime;
}
void initializeServo() {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_MIN_PULSE;  // Usar SERVO_MIN_PULSE en lugar de 1500
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // Mover el servo a 0 grados
    moveServoToAngle(0);
    HAL_Delay(1000);

    char debug[100];
    snprintf(debug, sizeof(debug), "Servo inicializado a 0 grados\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), HAL_MAX_DELAY);
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
