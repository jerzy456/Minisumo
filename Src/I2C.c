#include "I2C.h"
#include "hardware.h"
#include "stm32f4xx_hal.h"

int8_t I2C_write_multi(uint8_t addr, uint8_t index, void* pData, uint32_t size) {
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1,  addr,  index,
			I2C_MEMADD_SIZE_8BIT, pData, (uint16_t) size,100);
	return status;
}

int8_t I2C_read_multi(uint8_t addr, uint8_t index, void* pData, uint32_t size) {
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1,  addr,  index,
			I2C_MEMADD_SIZE_8BIT, pData, (uint16_t) size, 100);
	return status;
}

int8_t I2C_write_word(uint8_t addr, uint8_t index, uint16_t data) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t packet[] = { (data >> 8) & 0xFF, data & 0xFF };
	status = HAL_I2C_Mem_Write(&hi2c1,  addr, index,
			I2C_MEMADD_SIZE_8BIT, packet, 2,100);
	return status;
}
int8_t I2C_write_dword(uint8_t addr, uint8_t index, uint32_t data) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t packet[] = { (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8)
			& 0xFF, data & 0xFF };
	status = HAL_I2C_Mem_Write(&hi2c1,  addr, index,
			I2C_MEMADD_SIZE_8BIT, packet, 4,100);
	return status;
}

int8_t I2C_read_word(uint8_t addr, uint8_t index, uint16_t *data) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t packet[2];
	status = HAL_I2C_Mem_Read(&hi2c1,  addr, index,
			I2C_MEMADD_SIZE_8BIT, packet, 2,100);
	*data = (uint16_t) (packet[1] << 1) | packet[0];
	return status;
}
int8_t I2C_read_dword(uint8_t addr, uint8_t index, uint32_t *data) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t packet[4];
	status = HAL_I2C_Mem_Read(&hi2c1,  addr, index,
			I2C_MEMADD_SIZE_8BIT, packet, 2,100);
	*data = (uint32_t) (packet[3] << 3) | (packet[2] << 2) | (packet[1] << 1)
			| packet[0];
	return status;
}
