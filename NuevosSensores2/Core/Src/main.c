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
static int Msgindex;
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



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void process_gps_data(void);
void send_gps_data(void);
void Format_data(float time, float lat, char lat_dir, float lon, char long_dir);
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

	 //gps

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 while (1)
	 {
	   uint32_t currentTime = HAL_GetTick();
	   process_gps_data();

	   if (currentTime - lastTransmissionTime >= 1000)  // 1000 ms = 1 segundo
	   {
	     // Leer datos del GPS
		 send_gps_data();

	     // Leer datos del BMP280
	     float temperature, pressure, humidity;
	     bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);

	     // Leer datos del MPU9250
	     float accelData[3], gyroData[3], magData[3];
	     MPU9250_ReadAccel(&hi2c1, accelData);
	     MPU9250_ReadGyro(&hi2c1, gyroData);
	     MPU9250_ReadMag(&hi2c1, magData);

	     // Formar el mensaje con todos los datos
	     char buffer[512];
	     int len = snprintf(buffer, sizeof(buffer),
	         "Time=%02d:%02d:%05.2f, Lat=%.6f, Long=%.6f, "
	         "Temp=%.2f C, Press=%.2f Pa, Hum=%.2f%%, "
	         "AccX=%.2f,AccY=%.2f,AccZ=%.2f, "
	         "GyroX=%.2f,GyroY=%.2f,GyroZ=%.2f, "
	         "MagX=%.2f,MagY=%.2f,MagZ=%.2f\r\n",
	         hours, minutes, seconds, latitude, longitude,
	         temperature, pressure, humidity,
	         accelData[0], accelData[1], accelData[2],
	         gyroData[0], gyroData[1], gyroData[2],
	         magData[0], magData[1], magData[2]);

	     // Enviar el mensaje por UART
	     HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);

	     lastTransmissionTime = currentTime;
	   }
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
        char *ptr = strstr((char*)gpsBuffer, "$GPRMC");
        if (ptr != NULL)
        {
            if (sscanf(ptr, "$GPRMC,%f,A,%f,%c,%f,%c", &time, &latitude, &lat_dir, &longitude, &long_dir) == 5)
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
            else
            {
                // Si no se pudo parsear correctamente, invalidar los datos
                time = 0;
                latitude = 0;
                longitude = 0;
            }
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
        Format_data(time, latitude, lat_dir, longitude, long_dir);
    }
}

void Format_data(float time, float lat, char lat_dir, float lon, char long_dir)
{
    char Data[100];
    snprintf(Data, sizeof(Data), "\r\nTime=%02d:%02d:%05.2f, Lat=%.6f, Long=%.6f\r\n",
             hours, minutes, seconds, lat, lon);
    HAL_UART_Transmit(&huart1, (uint8_t*)Data, strlen(Data), 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        gpsDataReady = 1;
    }
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
