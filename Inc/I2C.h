/*
 * I2C.h
 *
 *  Created on: 15 gru 2016
 *      Author: jerzy
 */

#ifndef I2C_H_
#define I2C_H_
#include "stm32f4xx_hal.h"
#include "hardware.h"


int8_t I2C_write_multi(uint8_t addr, uint8_t index, void* pData, uint32_t size);

int8_t I2C_read_multi(uint8_t addr, uint8_t index, void* pData, uint32_t size);

int8_t I2C_write_word(uint8_t addr, uint8_t index, uint16_t data);
int8_t I2C_write_dword(uint8_t addr, uint8_t index, uint32_t data);

int8_t I2C_read_word(uint8_t addr, uint8_t index, uint16_t *data);
int8_t I2C_read_dword(uint8_t addr, uint8_t index, uint32_t *data);
#endif /* I2C_H_ */
