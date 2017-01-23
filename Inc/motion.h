/*
 * motion.h
 *
 *  Created on: 14 sty 2017
 *      Author: jerzy
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "pid.h"
#include "hardware.h"
#include "stm32f4xx_hal.h"

extern int16_t reqSpeedRight;
extern int16_t reqSpeedLeft;
extern uint8_t directionMot1;		//1 forward 0 backward
extern uint8_t directionMot2;
extern uint8_t directionMot3;
extern uint8_t directionMot4;

extern  pid_params pid_params_mot1;
extern  pid_params pid_params_mot2;
extern  pid_params pid_params_mot3;
extern  pid_params pid_params_mot4;

void Motion_init(void);
void Motion_tick(void);
void Motion_stop(void);


#endif /* MOTION_H_ */
