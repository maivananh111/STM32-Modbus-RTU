/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"

#include "STM_LOG.h"
#include "modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PT100_ADDR  		0x02
#define PT100_TEMP_REG  	0x00
#define PT100_RES_REG   	0x01
#define PT100_ADDR_REG 		0x02
#define PT100_BAUD_REG 		0x03

#define SHT_ADDR            0x01
#define SHT_TEMP_REG        0x0001
#define SHT_HUMI_REG        0x0002
#define SHT_ADDR_REG        0x0101
#define SHT_BAUD_REG        0x0102

#define RELAY_ADDR          0x01
#define RELAY_1_REG         0x00
#define RELAY_2_REG         0x01
#define RELAY_ON            0xFF00
#define RELAY_OFF           0x0000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
modbus mb_master(&huart2);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char *TAG = "Modbus";

uint8_t *RxData;

void STM_LOG_Set(char *buf);

float PT100_ReadTemperature(void);
float PT100_ReadResistance(void);
uint32_t Pt100_ReadBaudRate(void);
uint8_t PT100_ReadAddress(void);
void PT100_SetBaud(uint32_t NewBaud);
void PT100_SetAddress(uint8_t NewAddr);

float SHT_ReadTemperature(void);
float SHT_ReadHumidity(void);

void Write_Relay(uint8_t relay, uint16_t dt);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  STM_LOG_Init(STM_LOG_Set);
  STM_LOG(BOLD_GREEN, TAG, "LOG OKE. ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		SHT_ReadTemperature();
//		HAL_Delay(100);
//		SHT_ReadHumidity();
//		HAL_Delay(100);
		PT100_ReadTemperature();
		HAL_Delay(100);
		PT100_ReadResistance();
		HAL_Delay(1000);

	    Write_Relay(RELAY_1_REG, RELAY_ON);
	    HAL_Delay(100);
	    Write_Relay(RELAY_2_REG, RELAY_OFF);
	    HAL_Delay(100);


	    Write_Relay(RELAY_2_REG, RELAY_ON);
	    HAL_Delay(100);
	    Write_Relay(RELAY_1_REG, RELAY_OFF);
	    HAL_Delay(100);

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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void STM_LOG_Set(char *buf){
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 1000);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	mb_master.response_handler(huart);

}


float PT100_ReadTemperature(void){
	mb_master.send_request(PT100_ADDR, READ_HOLDING_REGISTERS, PT100_TEMP_REG, 0x01, true);

//	bool resp = mb_master.wait_for_response(1000);
//	if(resp == false) STM_LOG(SIMP_RED, TAG, "Fail to read temperature.");
	HAL_Delay(100);

	mb_master.get_response_datas(&RxData);
	float tmp = (float)((uint16_t)((RxData[3] << 8) | RxData[4]) / 10.0);
	STM_LOG(SIMP_RED, TAG, "PT100 Temperature = %f", tmp);

	return tmp;
}

float PT100_ReadResistance(void){
	mb_master.send_request(PT100_ADDR, READ_HOLDING_REGISTERS, PT100_RES_REG, 0x01, true);

	HAL_Delay(100);
//	bool resp = mb_master.wait_for_response(1000);
//	if(resp == false) STM_LOG(SIMP_RED, TAG, "Fail to read temperature.");

	mb_master.get_response_datas(&RxData);
	float res = (float)((uint16_t)((RxData[3] << 8) | RxData[4]) / 10.0);
	STM_LOG(SIMP_GREEN, TAG, "PT100 Resistance = %f", res);

	return res;
}
/*
uint32_t Pt100_ReadBaudRate(void){
	ModBus_Transmit(PT100_ADDR, PT100_FUNC_READ, PT100_BAUD_REG, 0x01);

	while(RxFlag == 0) HAL_Delay(1);
	RxFlag = 0;

	uint32_t Baud = 1200U << RxData[4];
	STM_LOG(SIMP_WHITE, TAG, TAG, "PT100 Current Baudrate: %d. ", Baud);
	return Baud;
}

uint8_t PT100_ReadAddress(void){
	ModBus_Transmit(PT100_ADDR, PT100_FUNC_READ, PT100_ADDR_REG, 0x01);

	while(RxFlag == 0) HAL_Delay(1);
	RxFlag = 0;

	uint8_t Addr = RxData[4];
	STM_LOG(SIMP_WHITE, TAG, TAG, "PT100 Current Address: 0x%02x. ", Addr);
	return Addr;
}

void PT100_SetBaud(uint32_t NewBaud){

}

void PT100_SetAddress(uint8_t NewAddr){

}
*/

float SHT_ReadTemperature(void){
	mb_master.send_request(SHT_ADDR, READ_INPUT_REGISTERS, SHT_TEMP_REG, 1, true);

	bool resp = mb_master.wait_for_response(1000);
	if(resp == false) STM_LOG(SIMP_RED, TAG, "Fail to read temperature.");

	mb_master.get_response_datas(&RxData);
	float tmp = (float)((uint16_t)((RxData[3] << 8) | RxData[4]) / 100.0);
	STM_LOG(SIMP_YELLOW, TAG, "SHT20 Temperature = %.2f", tmp);

	return tmp;
}

float SHT_ReadHumidity(void){
	mb_master.send_request(SHT_ADDR, READ_INPUT_REGISTERS, SHT_HUMI_REG, 1, true);

	bool resp = mb_master.wait_for_response(1000);
	if(resp == false) STM_LOG(SIMP_RED, TAG, "Fail to read humidity.");

	mb_master.get_response_datas(&RxData);
	float tmp = (float)((uint16_t)((RxData[3] << 8) | RxData[4]) / 100.0);
	STM_LOG(SIMP_CYAN, TAG, "SHT20 Humidity = %.2f", tmp);

	return tmp;
}


void Write_Relay(uint8_t relay, uint16_t dt){
	mb_master.send_request(RELAY_ADDR, WRITE_SINGLE_COIL, relay, dt, false);
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
