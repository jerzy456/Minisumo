/*
 * hardware.h
 *
 *  Created on: 23 gru 2016
 *      Author: jerzy
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_
#include "stm32f4xx_hal.h"



//Custom Hardware init functions
void HAL_Init_Peripherals(void);
void HAL_Set_Encoders(void);
void HAL_Set_PWM(void);


//HAL Init functions
void SystemClock_Config(void);
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_TIM2_Init(void);
void MX_TIM10_Init(void);
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

//HAL Hardware handlers
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart2;




#endif /* HARDWARE_H_ */
