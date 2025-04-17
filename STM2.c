/* USER CODE BEGIN Header */
 /**
 *************************************************************************
 *****
 * @file
 : main.c
 * @brief
 : Main program body
 *************************************************************************
 *****
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
 *************************************************************************
 *****
 */
 /* USER CODE END Header */
 /* Includes------------------------------------------------------------------*/
 #include "main.h"
 /* Private includes----------------------------------------------------------*/
 /* USER CODE BEGIN Includes */
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
 TIM_HandleTypeDef htim1;
 UART_HandleTypeDef huart2;
 /* USER CODE BEGIN PV */
 /* USER CODE END PV */
 /* Private function prototypes-----------------------------------------------*/
 void SystemClock_Config(void);
 static void MX_GPIO_Init(void);
 static void MX_USART2_UART_Init(void);
 static void MX_SPI1_Init(void);
 static void MX_TIM1_Init(void);
 /* USER CODE BEGIN PFP */
 void DWT_Init(void)
 {
 if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
 {
 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
 DWT->CYCCNT = 0;
 DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
 }
 }
 // 마이크로초 단위 지연 함수
void DWT_Delay(uint32_t us)
 {
 uint32_t startTick = DWT->CYCCNT;
 uint32_t delayTicks = us * (SystemCoreClock / 1000000);
 while (DWT->CYCCNT- startTick < delayTicks);
 }
 /* USER CODE END PFP */
 /* Private user code---------------------------------------------------------*/
 /* USER CODE BEGIN 0 */
 uint8_t inbuf[2];
 int target_angle = 0;
 int current_angle = 0;
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
 MX_SPI1_Init();
 MX_TIM1_Init();
 /* USER CODE BEGIN 2 */
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 HAL_GPIO_WritePin(LASER_GPIO_PORT, GPIO_PIN_5, GPIO_PIN_SET);
 /* USER CODE END 2 */
 char msg[100];
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while (1)
 {
 while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)==GPIO_PIN_SET) {}
 HAL_SPI_Receive(&hspi1, &inbuf[0], 1, 100);
 HAL_SPI_Receive(&hspi1, &inbuf[1], 1, 100);
 /* Display result */
 snprintf(msg, sizeof(msg), "\ninbuf[0] : %d , inbuf[1] : %d\r\n",
 inbuf[0], inbuf[1]);
 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 HAL_Delay(100);
 int target_angle = inbuf[0]+(inbuf[1]*256);
 // 각도 차이 계산
int angle_difference = target_angle- current_angle;
 if (angle_difference > 0)
 {
 // 시계 방향 회전
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000); // 시계 방향 펄스
(1ms)
 }
 else if (angle_difference < 0)
 {
 // 반시계 방향 회전
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1600); // 반시계 방향
펄스 (1.6ms)
 angle_difference =-angle_difference; // 절대값으로 변환
}
else
 {
 // 각도 차이가 0일 때, 목표 각도에 도달하여 정지
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1300); // 정지 펄스
continue; // 다음 loop로 이동
}
 // 목표 각도까지 회전 (1도씩 회전)
 while (angle_difference > 0)
 {
 DWT_Delay(5000); // 1도당 약 5ms 지연
angle_difference--; // 1도 회전 완료 처리
current_angle += (target_angle > current_angle) ? 1 :-1; // 현재
각도 업데이트
}
 // 목표 각도에 도달 후 정지
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1300); // 정지 펄스
current_angle = target_angle; // 현재 각도를 목표 각도로 업데이트
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
 hspi1.Init.Mode = SPI_MODE_SLAVE;
 hspi1.Init.Direction = SPI_DIRECTION_2LINES;
 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
 hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
 hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
 hspi1.Init.NSS = SPI_NSS_SOFT;
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
 htim1.Init.Prescaler = 71;
 htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim1.Init.Period = 19999;
 htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
 htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
 {
 Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) !=
 HAL_OK)
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
 if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) !=
 HAL_OK)
 {
 Error_Handler();
 }
 sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
 sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
 sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
 sBreakDeadTimeConfig.DeadTime = 0;
 sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
 sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
 sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
 if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) !=
 HAL_OK)
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
 /*Configure GPIO pin : B1_Pin */
 GPIO_InitStruct.Pin = B1_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pin : PC4 */
 GPIO_InitStruct.Pin = GPIO_PIN_4;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 /*Configure GPIO pin : PB5 */
 GPIO_InitStruct.Pin = GPIO_PIN_5;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStruct);
 /* EXTI interrupt init*/
 HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}
 /*USERCODEBEGIN4*/
 /*USERCODEEND4*/
 /**
 *@brief Thisfunctionisexecutedincaseoferroroccurrence.
 *@retvalNone
 */
 voidError_Handler(void)
 {
 /*USERCODEBEGINError_Handler_Debug*/
 /*UsercanaddhisownimplementationtoreporttheHALerrorreturn
 state*/
 __disable_irq();
 while(1)
 {
 }
 /*USERCODEENDError_Handler_Debug*/
 }
 #ifdef USE_FULL_ASSERT
 /**
 *@brief Reportsthenameofthesourcefileandthesourcelinenumber
 * wheretheassert_paramerrorhasoccurred.
 *@param file:pointertothesourcefilename
 *@param line:assert_paramerrorlinesourcenumber
 *@retvalNone
 */
 voidassert_failed(uint8_t*file,uint32_tline)
 {
 /*USERCODEBEGIN6*/
 /*Usercanaddhisownimplementationtoreportthefilenameandline
 number,
 ex:printf("Wrongparametersvalue:file%sonline%d\r\n",file,
 line)*/
 /*USERCODEEND6*/
 }
 #endif/*USE_FULL_ASSERT*/
