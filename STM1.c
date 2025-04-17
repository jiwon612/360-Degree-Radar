 /* USER CODE BEGIN Header */
 /**
 ***************************************************************************
 ***
 * @file
 : main.c
 * @brief
 : Main program body
 ***************************************************************************
 ***
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE
 file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ***************************************************************************
 ***
 */
 /* USER CODE END Header */
 /* Includes------------------------------------------------------------------*/
 #include "main.h"
 /* Private includes----------------------------------------------------------*/
 /* USER CODE BEGIN Includes */
 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include <math.h>
 /* USER CODE END Includes */
 /* Private typedef-----------------------------------------------------------*/
 /* USER CODE BEGIN PTD */
 /* USER CODE END PTD */
 /* Private define------------------------------------------------------------*/
 /* USER CODE BEGIN PD */
 /* USER CODE END PD */
 /* Private macro-------------------------------------------------------------*/
 /* USER CODE BEGIN PM */
 /* USER CODE END PM */
/* Private variables---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;
 UART_HandleTypeDef huart1;
 UART_HandleTypeDef huart2;
 /* USER CODE BEGIN PV */
 /* USER CODE END PV */
 /* Private function prototypes-----------------------------------------------*/
 void SystemClock_Config(void);
 static void MX_GPIO_Init(void);
 static void MX_USART2_UART_Init(void);
 static void MX_USART1_UART_Init(void);
 static void MX_SPI1_Init(void);
 /* USER CODE BEGIN PFP */
 /* USER CODE END PFP */
 /* Private user code---------------------------------------------------------*/
 /* USER CODE BEGIN 0 */
 uint8_t outbuf[2];
 typedef struct {
 float distance; // Distance from the servo to the object (b)
 float alpha;
 object
 int num;
 } ServoCalcResult;
 ServoCalcResult calculate_alpha_and_distance(int servo_num, float
 sensor_distance, float sensor_angle, float servo_to_sensor_distance) {
 ServoCalcResult result;
 result.num = servo_num;
 // Adjust angle based on sensor orientation
 float effective_angle = (sensor_angle >= 90.0) ? 270.0- sensor_angle :
 90.0 + sensor_angle;
 // Convert angle to radians
 float radian_angle = effective_angle * (M_PI / 180.0);
 // Calculate distance to the object using the law of cosines
 float b = sqrt(pow(servo_to_sensor_distance, 2) + pow(sensor_distance,
 2)
2 * servo_to_sensor_distance * sensor_distance *
 cos(radian_angle));
 result.distance = b;
 // Calculate alpha (angle)
 float alpha = acos((pow(servo_to_sensor_distance, 2) + pow(b, 2)
pow(sensor_distance, 2)) /
 (2 * servo_to_sensor_distance * b)) * (180.0 /
 M_PI);
 // Determine direction based on sensor angle
 result.alpha = (sensor_angle < 90.0) ? 360.0- alpha : alpha;
 return result;
 }
 void UART_ReceiveFloat(UART_HandleTypeDef *huart, float *value) {
 char rx_buffer[10];
 // Angle the servo motor should face towards the
int i = 0;
 char received_char;
 // Clear buffer
 memset(rx_buffer, 0, sizeof(rx_buffer));
 // Receive characters until null terminator or Enter key
 while (1) {
 HAL_UART_Receive(huart, (uint8_t*)&received_char, 1, HAL_MAX_DELAY);
 // Exit if null terminator is received
 if (received_char == '\0') {
 break;
 }
 // Add character to buffer
 if (i < sizeof(rx_buffer)- 1) {
 rx_buffer[i++] = received_char;
 }
 }
 // Add null-terminator at end of buffer
 rx_buffer[i] = '\0';
 // Convert buffer to float
 *value = atof(rx_buffer);
 }
 /* USER CODE END 0 */
 /**
 * @brief The application entry point.
 * @retval int
 */
 int main(void)
 {
 /* USER CODE BEGIN 1 */
 /* USER CODE END 1 */
 /* MCU
 Configuration--------------------------------------------------------*/
 /* Reset of all peripherals, Initializes the Flash interface and the
 Systick. */
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
 MX_USART1_UART_Init();
 MX_SPI1_Init();
 /* USER CODE BEGIN 2 */
 float servo_num1;
 float sensor_distance1 = 0.0;
 float sensor_angle1 = 0.0;
 float servo_num2;
 float sensor_distance2 = 0.0;
float sensor_angle2 = 0.0;
 float servo_num3;
 float sensor_distance3 = 0.0;
 float sensor_angle3 = 0.0;
 float servo_to_sensor_distance = 17.0;
 char msg[100];
 /* USER CODE END 2 */
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while (1) {
 /* Receive servo number */
 UART_ReceiveFloat(&huart1, &servo_num1);
 /* Receive distance */
 UART_ReceiveFloat(&huart1, &sensor_distance1);
 /* Receive angle */
 UART_ReceiveFloat(&huart1, &sensor_angle1);
 /* Receive servo number */
 UART_ReceiveFloat(&huart1, &servo_num2);
 /* Receive distance */
 UART_ReceiveFloat(&huart1, &sensor_distance2);
 /* Receive angle */
 UART_ReceiveFloat(&huart1, &sensor_angle2);
 /* Receive servo number */
 UART_ReceiveFloat(&huart1, &servo_num3);
 /* Receive distance */
 UART_ReceiveFloat(&huart1, &sensor_distance3);
 /* Receive angle */
 UART_ReceiveFloat(&huart1, &sensor_angle3);
 /* Perform calculation */
 ServoCalcResult result1 = calculate_alpha_and_distance(1,
 sensor_distance1, sensor_angle1, servo_to_sensor_distance);
 ServoCalcResult result2 = calculate_alpha_and_distance(2,
 sensor_distance2, sensor_angle2, servo_to_sensor_distance);
 ServoCalcResult result3 = calculate_alpha_and_distance(3,
 sensor_distance3, sensor_angle3, servo_to_sensor_distance);
 if (sensor_angle2 < 90) {
 result2.alpha = result2.alpha- 240;
 }
 else {
 result2.alpha = result2.alpha + 120;
 }
 if (sensor_angle3 < 90) {
 result3.alpha = result3.alpha- 120;
 }
 else {
 result3.alpha = result3.alpha + 240;
 }
 ServoCalcResult min_result = (result1.distance > result2.distance)
 ? ((result2.distance < result3.distance) ? result2 : result3)
 : ((result1.distance < result3.distance) ? result1 : result3);
 uint16_t value = min_result.alpha;
 outbuf[1]=(value >> 8)&0xFF;
outbuf[0]= (value & 0xFF);
 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
 HAL_SPI_Transmit(&hspi1, &outbuf[0], 1, 100);
 HAL_Delay(1);
 HAL_SPI_Transmit(&hspi1, &outbuf[1], 1, 100);
 HAL_Delay(1);
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
 snprintf(msg, sizeof(msg), "\nvalue : %d\r\n", value);
 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 snprintf(msg, sizeof(msg), "\noutbuf[0]: %d, outbuf[1]: %d\r\n",
 outbuf[0], outbuf[1]);
 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 /* Display result */
 snprintf(msg, sizeof(msg), "\nCalculated Distance1: %.2f cm, Alpha:
 %.2f degrees\r\n", result1.distance, result1.alpha);
 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 snprintf(msg, sizeof(msg), "Calculated Distance2: %.2f cm, Alpha: %.2f
 degrees\r\n", result2.distance, result2.alpha);
 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 snprintf(msg, sizeof(msg), "Calculated Distance3: %.2f cm, Alpha: %.2f
 degrees\r\n", result3.distance, result3.alpha);
 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 snprintf(msg, sizeof(msg), "Nearest Servo num: %d, Calculated
 Distance_min: %.2f cm, Alpha: %.2f degrees\r\n", min_result.num,
 min_result.distance, min_result.alpha);
 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 }
 /* USER CODE END WHILE */
 /* USER CODE BEGIN 3 */
 }
 /* USER CODE END 3 */
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
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
 RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
 RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
 huart1.Init.StopBits = UART_STOPBITS_2;
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
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
 /*Configure GPIO pin : B1_Pin */
 GPIO_InitStruct.Pin = B1_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pin : PC4 */
 GPIO_InitStruct.Pin = GPIO_PIN_4;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 /* EXTI interrupt init*/
 HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
 }
 /* USER CODE BEGIN 4 */
 /* USER CODE END 4 */
 /**
 * @brief This function is executed in case of error occurrence.
 * @retval None
 */
 void Error_Handler(void)
 {
 /* USER CODE BEGIN Error_Handler_Debug */
 /* User can add his own implementation to report the HAL error return
 state */
 __disable_irq();
 while (1)
 {
 }
 /* USER CODE END Error_Handler_Debug */
 }
 #ifdef USE_FULL_ASSERT
 /**
 * @brief Reports the name of the source file and the source line number
 *
 where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
 void assert_failed(uint8_t *file, uint32_t line)
{
 /* USER CODE BEGIN 6 */
 /* User can add his own implementation to report the file name and line
 number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
 line) */
 /* USER CODE END 6 */
 }
 #endif /* USE_FULL_ASSERT */
