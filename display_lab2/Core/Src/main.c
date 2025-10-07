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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "nmea_parse.h"
#include <stdarg.h>
#include "ili9341.h"
#include "ili9341_touch.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RxBuffer_SIZE 64  //configure uart receive buffer size
#define CLI_SIZE 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

uint16_t oldPos = 0;
uint16_t newPos = 0;
uint8_t RxBuffer[RxBuffer_SIZE];
uint8_t DataBuffer[DataBuffer_SIZE];

uint8_t cli_byte;
char cli_buffer[CLI_SIZE];
uint8_t cli_index = 0;
uint8_t cli_ready = 0;
char command[CLI_SIZE];
uint8_t raw_mode_on = 0;
uint32_t lastUpdate = 0;
char device_id_str[16];

GPS myData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint8_t validate_checksum(char* sentence);
void parse_RMC(char* sentence);
void parse_GGA(char* sentence);
float nmea_to_decimal(char* coord, char hemisphere);
void process_nmea_sentence(char* sentence);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void process_commands(char* cmd){
	if(strcmp(cmd, "VER") == 0){
		char str[128];
		sprintf(str, "FW v1.0, build: %s %s, UART2 9600\r\n", __DATE__, __TIME__);
		HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);
		cli_ready = 0;
	}else if (strcmp(cmd, "GET POS") == 0) {
		nmea_parse(&myData, DataBuffer);
	    char buf[32];

	    // Latitude
	    HAL_UART_Transmit(&huart2, (uint8_t*)"Lat: ", 5, 100);
	    sprintf(buf, "%f\r\n", myData.latitude);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

	    // Latitude side
	    HAL_UART_Transmit(&huart2, (uint8_t*)"Lat side: ", 10, 100);
	    sprintf(buf, "%c\r\n", myData.latSide);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

	    // Longitude
	    HAL_UART_Transmit(&huart2, (uint8_t*)"Lon: ", 5, 100);
	    sprintf(buf, "%f\r\n", myData.longitude);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

	    // Longitude side
	    HAL_UART_Transmit(&huart2, (uint8_t*)"Lon side: ", 10, 100);
	    sprintf(buf, "%c\r\n", myData.lonSide);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

	    // Fix
	    HAL_UART_Transmit(&huart2, (uint8_t*)"Fix: ", 5, 100);
	    sprintf(buf, "%d\r\n", myData.fix);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

	    // Satellite count
	    HAL_UART_Transmit(&huart2, (uint8_t*)"Sats: ", 6, 100);
	    sprintf(buf, "%d\r\n", myData.satelliteCount);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
		cli_ready = 0;
    } else if (strcmp(cmd, "GET TIME") == 0) {
    	nmea_parse(&myData, DataBuffer);
        char str[64];
        sprintf(str, "UTC:%s, stale:%s\r\n", myData.lastMeasure,
            (myData.fix ? "no" : "yes"));
        HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
		cli_ready = 0;
    } else if (strncmp(cmd, "RAW ", 4) == 0) {
        if (strcmp(cmd+4, "ON") == 0) raw_mode_on = 1;
        HAL_UART_Transmit(&huart2, (uint8_t*)"RAW mode updated\r\n", 17, 100);
        raw_mode_on = 1;
        while(raw_mode_on){
        	nmea_parse(&myData, DataBuffer);
        }
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Unknown command\r\n", 17, 100);
		cli_ready = 0;
    }
}

uint32_t get_unique_id(void) {
	uint32_t *uid = (uint32_t*)0x1FFF7A10; //UID address in STM32-nucleoF446re
    return uid[0] ^ uid[1] ^ uid[2]; // XOR all three 32 bit parts
}
void display_gps_data(GPS *data) {
    char buf[64];

    // ID
    sprintf(buf, "ID: %s", device_id_str);
    ILI9341_WriteString(10, 40, buf, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);

    // Latitude
    sprintf(buf, "LAT: %.6f", data->latitude);
    ILI9341_WriteString(10, 70, buf, Font_11x18, ILI9341_GREEN, ILI9341_BLACK);

    sprintf(buf, "LAT side: %c", data->latSide);
    ILI9341_WriteString(10, 90, buf, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

    // Longitude
    sprintf(buf, "LON: %.6f", data->longitude);
    ILI9341_WriteString(10, 120, buf, Font_11x18, ILI9341_GREEN, ILI9341_BLACK);

    sprintf(buf, "LON side: %c", data->lonSide);
    ILI9341_WriteString(10, 140, buf, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

    // UTC Time
    sprintf(buf, "UTC: %s", data->lastMeasure);
    ILI9341_WriteString(10, 170, buf, Font_11x18, ILI9341_WHITE, ILI9341_BLACK);

    // Satellites
    sprintf(buf, "Sats: %d", data->satelliteCount);
    ILI9341_WriteString(10, 200, buf, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1){
		oldPos = newPos; //keep track of the last position in the buffer
		if(oldPos + Size > DataBuffer_SIZE){ //if the buffer is full, parse it, then reset the buffer

			uint16_t datatocopy = DataBuffer_SIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)DataBuffer+oldPos, RxBuffer, datatocopy);  // copy data in that remaining space

			oldPos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)DataBuffer, (uint8_t *)RxBuffer+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}
		else{
			memcpy((uint8_t *)DataBuffer+oldPos, RxBuffer, Size); //copy received data to the buffer
			newPos = Size+oldPos; //update buffer position

		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)RxBuffer, RxBuffer_SIZE); //re-enable the DMA interrupt
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //disable the half transfer interrupt
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // твій порт
    {
    	raw_mode_on = 0;
		cli_ready = 0;
    	HAL_UART_Transmit(&huart2, &cli_byte, 1, 1000);
        if (cli_byte == '\r' || cli_byte == '\n')
        {
            cli_buffer[cli_index] = '\0';
            memcpy(command, cli_buffer, cli_index + 1);
            HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
            cli_ready = 1;
            cli_index = 0;

        }
        else
        {
            if (cli_index < CLI_SIZE - 1){

                cli_buffer[cli_index++] = cli_byte;
            }
            else
				{
					cli_buffer[cli_index] = '\0';

					char msg[] = "\r\nInvalid command\r\n";
					HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

					cli_index = 0;
				}

        }

    }
    HAL_UART_Receive_IT(&huart2, &cli_byte, 1);
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
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize ILI9341 display
  ILI9341_Init();

  // Fill screen with red
  ILI9341_FillScreen(ILI9341_RED);
  HAL_Delay(500);

  // Clear and show interface
  uint32_t node_id = get_unique_id();
  sprintf(device_id_str, "0x%08lX", node_id);

  ILI9341_FillScreen(ILI9341_BLACK);
  ILI9341_WriteString(10, 10, "GPS ready", Font_16x26, ILI9341_YELLOW, ILI9341_BLACK);
  HAL_Delay(500);


  char msg[] = "GPS NMEA Parser Started\r\n";
  int8_t msg_len = strlen(msg);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, msg_len, HAL_MAX_DELAY);
  // Start UART DMA reception
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)RxBuffer, RxBuffer_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  int Serialcnt = 0;
  HAL_UART_Receive_IT(&huart2, &cli_byte, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_GetTick() - lastUpdate >= 1000) {
	      nmea_parse(&myData, DataBuffer);
	      display_gps_data(&myData);
	      lastUpdate = HAL_GetTick();
	  }

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
