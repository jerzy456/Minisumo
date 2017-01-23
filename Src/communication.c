/*
 * communication.c
 *
 *  Created on: 14 sty 2017
 *      Author: jerzy
 */

#include "communication.h"

uint8_t Received[UART2_RX_bufferlen];

void print(const char *format, ...) {

    	// Generate output
        va_list argptr;
        va_start(argptr, format);
        vsprintf((char*)buffer_print, format, argptr);
        va_end(argptr);

        HAL_UART_Transmit_IT(&huart2, (uint8_t *)buffer_print, strlen((const char*)buffer_print));
    	// Wait for UART to finish

}
