#ifndef INC_SERVO_H_
#define INC_SERVO_H_


#include "stm32f4xx_hal.h"

#include "define.h"


extern TIM_HandleTypeDef htim2;
extern void Error_Handler();


#define SERVO_TIMER               htim2
#define SERVO_CHANNEL             TIM_CHANNEL_2

#define SERVO_TIMER_FREQUENCY     50                                  // Hz
#define SERVO_TIMER_PRESCALER     625                                 //
#define SERVO_TIMER_PERIOD        (1000000 / SERVO_TIMER_FREQUENCY)   // Hz
#define SERVO_TIMER_RESOLUTION    2048                                //

#define SERVO_MAX_ANGLE           180                                 // degrees
#define SERVO_MIN_IMP             500                                 // us
#define SERVO_MAX_IMP             2450                                // us
#define SERVO_DELTA_IMP           ( SERVO_MAX_IMP - SERVO_MIN_IMP )   // us


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void SERVO_TimerInit(void);
uint16_t SERVO_getCCR(uint16_t angle);


#define  SERVO_setAngle(a)         __HAL_TIM_SET_COMPARE( &SERVO_TIMER, SERVO_CHANNEL, SERVO_getCCR(a) );
#define  SERVO_Enable()            HAL_TIM_PWM_Start(&SERVO_TIMER, SERVO_CHANNEL);
#define  SERVO_Disable()           HAL_TIM_PWM_Stop(&SERVO_TIMER, SERVO_CHANNEL);


#endif /* INC_SERVO_H_ */
