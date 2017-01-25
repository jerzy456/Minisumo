/*
 * motion.c
 *
 *  Created on: 14 sty 2017
 *      Author: jerzy
 */


#include "motion.h"

int16_t reqSpeedRight = 0;
int16_t reqSpeedLeft = 0;
uint8_t directionMot1 = 1;		//1 forward 0 backward
uint8_t directionMot2 = 1;
uint8_t directionMot3 = 1;
uint8_t directionMot4 = 1;

pid_params pid_params_mot1;
pid_params pid_params_mot2;
pid_params pid_params_mot3;
pid_params pid_params_mot4;



void Motion_init(void){
	//init motor pids values
	PID_init(18, 0.74, 0, 100, &pid_params_mot1);
	PID_init(25, 2, 0, 100, &pid_params_mot2);
	PID_init(18, 0.74, 0, 100, &pid_params_mot3);
	PID_init(25, 2, 0, 100, &pid_params_mot4);

	//Set half of ARR register value as default timers CNT registers value
	HAL_Set_Encoders();
	//Set default PWM value  0
	HAL_Set_PWM();
	//Motor control interrupt ON
	HAL_TIM_Base_Start_IT(&htim10);


}

void Motion_tick(void){
	float pid1 = 0;
			float pid2 = 0;
			float pid3 = 0;
			float pid4 = 0;

			int16_t duty1 = 0;
			int16_t duty2 = 0;
			int16_t duty3 = 0;
			int16_t duty4 = 0;

			static int16_t actPulse1[] = { 0, 0 };
			static int16_t actPulse2[] = { 0, 0 };
			static int16_t actPulse3[] = { 0, 0 };
			static int16_t actPulse4[] = { 0, 0 };

			actPulse1[1] = actPulse1[0];
			actPulse2[1] = actPulse2[0];
			actPulse2[1] = actPulse3[0];
			actPulse4[1] = actPulse4[0];
			//read encoders, fix mistakes with wiring and difference in encoders
			actPulse1[0] = TIM1->CNT;
			actPulse2[0] = ((TIM3->ARR) - (TIM3->CNT));	//change polarisation //motor 100:1
			actPulse3[0] = (TIM4->ARR) - (TIM4->CNT);		//change polarisation
			actPulse4[0] = ((TIM8->CNT));//change polarisation and apply gain factor for diffrent encoder

			TIM1->CNT = (TIM1->ARR) / 2; //set default value of half of  the maximal value
			TIM3->CNT = (TIM3->ARR) / 2;
			TIM4->CNT = (TIM4->ARR) / 2;
			TIM8->CNT = (TIM8->ARR) / 2;

			//prepare data for pid functions
			actPulse1[0] -= (TIM1->ARR) / 2;
			actPulse2[0] -= (TIM3->ARR) / 2;
			actPulse3[0] -= (TIM4->ARR) / 2;
			actPulse4[0] -= (TIM8->ARR) / 2;

			actPulse1[0] = (actPulse1[0] + actPulse1[1]) / 2;
			actPulse2[0] = (actPulse2[0] + actPulse2[1]) / 2;
			actPulse3[0] = (actPulse3[0] + actPulse3[1]) / 2;
			actPulse4[0] = (actPulse4[0] + actPulse4[1]) / 2;

			//calculate pids

			if (reqSpeedLeft != 0) {
				pid1 = PID_calculate((float) reqSpeedLeft, (float) actPulse1[0],
						&pid_params_mot1);
				pid4 = PID_calculate((float) 1.4 * reqSpeedLeft,
						(float) actPulse4[0], &pid_params_mot4);

			} else {
				pid1 = 0;
				pid4 = 0;
			}

			if (reqSpeedRight != 0) {

				pid2 = PID_calculate((float) 1.4 * reqSpeedRight,
						(float) actPulse2[0], &pid_params_mot2);
				pid3 = PID_calculate((float) reqSpeedRight, (float) actPulse3[0],
						&pid_params_mot3);
			} else {
				pid2 = 0;
				pid3 = 0;
			}

			//fix duty value to positive values only and set direction
			if (pid1 < 0) {
				duty1 = (-1) * pid1;
				directionMot1 = 0;
			} else {
				directionMot1 = 1;
				duty1 = pid1;
			}
			if (pid2 < 0) {
				duty2 = (-1) * pid2;
				directionMot2 = 0;
			} else {
				directionMot2 = 1;
				duty2 = pid2;
			}
			if (pid3 < 0) {
				duty3 = (-1) * pid3;
				directionMot3 = 0;
			} else {
				directionMot3 = 1;
				duty3 = pid3;
			}
			if (pid4 < 0) {
				duty4 = (-1) * pid4;
				directionMot4 = 0;
			} else {
				directionMot4 = 1;
				duty4 = pid4;
			}

			//set direction

			if (directionMot3 == 1) {
				HAL_GPIO_WritePin(IN4_A_GPIO_Port, IN4_A_Pin, 0);		//FW
				HAL_GPIO_WritePin(IN4_B_GPIO_Port, IN4_B_Pin, 1);
			} else {
				HAL_GPIO_WritePin(IN4_A_GPIO_Port, IN4_A_Pin, 1);		//BW
				HAL_GPIO_WritePin(IN4_B_GPIO_Port, IN4_B_Pin, 0);
			}
			if (directionMot4 == 1) {
				HAL_GPIO_WritePin(IN3_A_GPIO_Port, IN3_A_Pin, 0);		//FW
				HAL_GPIO_WritePin(IN3_B_GPIO_Port, IN3_B_Pin, 1);
			} else {
				HAL_GPIO_WritePin(IN3_A_GPIO_Port, IN3_A_Pin, 1);		//BW
				HAL_GPIO_WritePin(IN3_B_GPIO_Port, IN3_B_Pin, 0);
			}

			if (directionMot2 == 1) {

				HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, 0);		//FW
				HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, 1);
			} else {

				HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, 1);		//BW
				HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, 0);
			}
			if (directionMot1 == 1) {
				HAL_GPIO_WritePin(IN2_A_GPIO_Port, IN2_A_Pin, 1);		//FW
				HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, 0);
			} else {
				HAL_GPIO_WritePin(IN2_A_GPIO_Port, IN2_A_Pin, 0);		//BW
				HAL_GPIO_WritePin(IN2_B_GPIO_Port, IN2_B_Pin, 1);
			}

			//apply pwm
			TIM2->CCR1 = duty4;
			TIM2->CCR2 = duty2;
			TIM2->CCR3 = duty1;
			TIM2->CCR4 = duty3;

}
void Motion_stop(void){
	reqSpeedRight=0;
	reqSpeedRight=0;
}
