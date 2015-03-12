/*
 * i2c_functions.h
 *
 *  Created on: 27/11/2014
 *      Author: Fernando1
 */

#ifndef I2C_FUNCTIONS_H_
#define I2C_FUNCTIONS_H_

#include <stdint.h>
#include <stdlib.h>

int I2CGetErrorsCount(void);
void I2CInit(void);
void I2CWrite(uint8_t data, uint32_t cmd);
uint8_t I2CRead(uint32_t cmd);
uint8_t I2CReadMulti(uint8_t add, uint8_t *buf, size_t size);
size_t I2CReadMultiRegisters(uint8_t add, uint8_t reg, uint8_t *buf, size_t size);
void I2CWriteRegister(uint8_t add, uint8_t reg, uint8_t val);
uint8_t I2CReadRegister(uint8_t add, uint8_t reg);
void SwapEndianness(uint8_t *buf, size_t size);

#endif /* I2C_FUNCTIONS_H_ */
