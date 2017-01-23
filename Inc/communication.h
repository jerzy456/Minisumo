/*
 * communication.h
 *
 *  Created on: 14 sty 2017
 *      Author: jerzy
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#define UART2_TX_bufferlen 60
#define UART2_RX_bufferlen 4


#include "stm32f4xx_hal.h"
#include "hardware.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"

extern uint8_t Received[UART2_RX_bufferlen];
uint8_t buffer_print[UART2_TX_bufferlen];

void print(const char *format, ...);

#endif /* COMMUNICATION_H_ */
