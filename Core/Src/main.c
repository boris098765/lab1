/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* -------------- */

#define TIM8_PRESC            ( 125 - 1 )

#define CRT_GAMMA             2.2
#define CRT_MAX               255
#define CRT_TABLE_LEN         ( CRT_MAX * 2 )

#define MAIN_LED_FLASH_FREQ   120

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_up;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

#define LED_TIMER                 htim4
#define LED_CHANNEL               TIM_CHANNEL_1

#define LED_SET_TIMER             htim8

/* -------------- */

uint16_t LED_Period;
float CRT_K;
float CRT_Gamma;

uint16_t led_brightness_table[CRT_TABLE_LEN];

/* -------------- */

#define MSG_LEN            7
#define FULL_MSG_LEN       (MSG_LEN + 2)

uint8_t rx_buffer[FULL_MSG_LEN];
uint8_t msg_buffer[FULL_MSG_LEN];
volatile uint8_t msg_received_flug = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void setup_CRT();
uint16_t get_CRT(uint16_t val);
void calc_brightness_table(uint16_t *table);
void set_blink_frequency(TIM_HandleTypeDef *htim, uint32_t freq);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void USART_DMA_SendString(UART_HandleTypeDef *huart, const char *str);

void messageProcessing(uint8_t* message);
void LEDProcessing(uint8_t func, uint32_t data);
void ServoProcessing(uint8_t func, uint32_t data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void calc_CRT_K() {
	CRT_K = ( CRT_MAX / (pow(CRT_MAX, CRT_Gamma)) );
}

void setup_CRT() {
	CRT_Gamma = CRT_GAMMA;
	calc_CRT_K();
}

uint16_t get_CRT(uint16_t val) {
	return (uint16_t)( CRT_K * pow(val, CRT_Gamma) );
}

void calc_brightness_table(uint16_t *table) {
	for (uint16_t i=0; i<CRT_MAX; i++) {
		uint16_t val = (uint16_t)get_CRT(i);
		table[i] = val;
		table[CRT_TABLE_LEN-i-1] = val;
	}
}

void set_blink_frequency(TIM_HandleTypeDef *htim, uint32_t freq) {
    uint32_t period = 0;

    freq *= CRT_TABLE_LEN;
    freq /= 60;

    if (freq > 0) {
        uint32_t tick_freq = MAIN_CLOCK / (TIM8_PRESC + 1);
        period = tick_freq / freq;
    }

    __HAL_TIM_SET_AUTORELOAD(htim, period);
}

#define LED_Enable()      HAL_TIM_PWM_Start(&htim4, LED_CHANNEL);
#define LED_Disable()     HAL_TIM_PWM_Stop(&htim4, LED_CHANNEL);

/* -------------- */

#define USART_DMA_RX_Init()     HAL_UART_Receive_DMA(&huart2, rx_buffer, FULL_MSG_LEN);

void USART_DMA_SendString(UART_HandleTypeDef *huart, const char *str) {
    uint16_t len = strlen(str);
    HAL_UART_Transmit_DMA(huart, (uint8_t *)str, len);
}
#define uart_send(str) USART_DMA_SendString(&huart2, str)

/* -------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART2) return;

    memmove(msg_buffer, rx_buffer, FULL_MSG_LEN);
    msg_received_flug = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART2) return;

    //
}

/* -------------- */

uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

void messageProcessing(uint8_t *message) {
	uint8_t first_byte  = message[0];
	uint8_t target_byte = message[1];
	uint8_t func_byte   = message[2];
	uint32_t data_bytes = (uint32_t)message[3] << 24 |
			              (uint32_t)message[4] << 16 |
						  (uint32_t)message[5] << 8 |
						  (uint32_t)message[6] << 0;
	uint8_t crc_byte    = message[7];

	if (first_byte != 0xFF) return;
	if (crc_byte != crc8(message, 7)) return;

	switch (target_byte) {
	case 0x00:
		LEDProcessing(func_byte, data_bytes);
		break;
	case 0x01:
		ServoProcessing(func_byte, data_bytes);
		break;
	}
}

void LEDProcessing(uint8_t func, uint32_t data) {
	switch (func) {
	case 0x00: // ON / OFF
		if (data & 0x01) LED_Enable() else LED_Disable();
		break;
	case 0x01: // Frequency
		set_blink_frequency(&LED_SET_TIMER, data);
		break;
	case 0x02: // Gamma
		CRT_Gamma = (float)data/10;
		calc_CRT_K();
		calc_brightness_table(led_brightness_table);
		break;
	}
}

void ServoProcessing(uint8_t func, uint32_t data) {
	switch (func) {
	case 0x00: // Угол
		SERVO_setAngle((uint16_t)data);
		break;
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  SERVO_TimerInit();

  USART_DMA_RX_Init();

  setup_CRT();
  calc_brightness_table(led_brightness_table);

  SERVO_Enable();

  LED_Enable();
  HAL_DMA_Start(&hdma_tim8_up, (uint32_t)led_brightness_table, (uint32_t)&TIM4->CCR1, CRT_TABLE_LEN);
  HAL_TIM_Base_Start(&LED_SET_TIMER);

  uart_send("\nПогнали, йоптыть!\n\r");
  HAL_Delay(500);
  uart_send("Работает?\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  set_blink_frequency(&LED_SET_TIMER, MAIN_LED_FLASH_FREQ);
  SERVO_setAngle(0);

  while (1) {
	if (!msg_received_flug) continue;

	msg_received_flug = 0;
	messageProcessing(msg_buffer);
	USART_DMA_RX_Init();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 25-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 256-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

static void MX_TIM8_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  LED_SET_TIMER.Instance = TIM8;
  LED_SET_TIMER.Init.Prescaler = 125-1;
  LED_SET_TIMER.Init.CounterMode = TIM_COUNTERMODE_UP;
  LED_SET_TIMER.Init.Period = 1003;
  LED_SET_TIMER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  LED_SET_TIMER.Init.RepetitionCounter = 0;
  LED_SET_TIMER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&LED_SET_TIMER) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&LED_SET_TIMER, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&LED_SET_TIMER, &sMasterConfig) != HAL_OK) Error_Handler();

  TIM8->DIER |= TIM_DIER_UDE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ERROR_LED_Pin */
  GPIO_InitStruct.Pin = ERROR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ERROR_LED_GPIO_Port, &GPIO_InitStruct);

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
  while (1) {
	HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
	HAL_Delay(500);
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
