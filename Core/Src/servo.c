#include "servo.h"


void SERVO_TimerInit(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Config Timer

  SERVO_TIMER.Instance = TIM2;
  SERVO_TIMER.Init.Prescaler = SERVO_TIMER_PRESCALER - 1;
  SERVO_TIMER.Init.CounterMode = TIM_COUNTERMODE_UP;
  SERVO_TIMER.Init.Period = SERVO_TIMER_RESOLUTION - 1;
  SERVO_TIMER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  SERVO_TIMER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&SERVO_TIMER) != HAL_OK) Error_Handler();

  // Config Clock

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&SERVO_TIMER, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&SERVO_TIMER) != HAL_OK) Error_Handler();

  // Config Trigger

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&SERVO_TIMER, &sMasterConfig) != HAL_OK) Error_Handler();

  // Config Channel

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&SERVO_TIMER, &sConfigOC, SERVO_CHANNEL) != HAL_OK) Error_Handler();

  // Config GPIO

  HAL_TIM_MspPostInit(&SERVO_TIMER);

}

uint16_t SERVO_getCCR(uint16_t angle) {
	if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

	uint16_t imp = SERVO_MIN_IMP + ( SERVO_DELTA_IMP * angle / SERVO_MAX_ANGLE );
	uint16_t ccr = (imp * SERVO_TIMER_RESOLUTION) / SERVO_TIMER_PERIOD - 1;

	if (ccr >= SERVO_TIMER_RESOLUTION) ccr = SERVO_TIMER_RESOLUTION - 1;

	return ccr;
}
