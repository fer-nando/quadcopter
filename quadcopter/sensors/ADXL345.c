/*
 * ADXL345.c
 *
 * I2C Accelerometer ADXL345
 *
 * I2C adress: 0x3A (8bit)    0x1D (7bit)
 * Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
 * principle:
 *  1) CS PIN must be linked to VCC to select the I2C mode
 *  2) SD0 PIN must be linked to VCC to select the right I2C adress
 *  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
 *  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#include <inc/hw_types.h>
#include <driverlib/i2c.h>
#include "../utils/delay.h"
#include "../utils/linalg.h"
#include "../utils/i2c_functions.h"
#include "ADXL345.h"

extern uint8_t rawADC[6];

int accADC[3];
int accZero[3];

//*****************************************************************************
//
//
//
//*****************************************************************************
void ADXL345_Init() {
  // Enable measurements
  I2CWriteRegister(ADXL345_ADDRESS, ADXL345_REG_POWER_CTL, 1 << 3);
  // Set full range and +/- 16g-range
  I2CWriteRegister(ADXL345_ADDRESS, ADXL345_REG_DATA_FORMAT, 0x0B);
  // Set rate 400hz and bandwidth 200hz
  I2CWriteRegister(ADXL345_ADDRESS, ADXL345_REG_BW_RATE, 0x0C); // 0x09 -> 50Hz
  DelayMs(5);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void ADXL345_ReadADC() {
  I2CReadMultiRegisters(ADXL345_ADDRESS, 0x32, rawADC, 6);

  ACC_ORIENTATION( ((int16_t)(rawADC[1]<<8) | (int16_t)rawADC[0]),
      ((int16_t)(rawADC[3]<<8) | (int16_t)rawADC[2]),
      ((int16_t)(rawADC[5]<<8) | (int16_t)rawADC[4]));
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void ADXL345_SetCalibration(int * zero) {
  accZero[0] = zero[0];
  accZero[1] = zero[1];
  accZero[2] = zero[2];
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void ADXL345_GetCalibration(int * zero) {
  zero[0] = accZero[0];
  zero[1] = accZero[1];
  zero[2] = accZero[2];
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void ADXL345_Calibrate() {
  int i;
  int accSum[3] = { 0, 0, 0 };

  for (i = 0; i < 100; i++) {
    ADXL345_ReadADC();
    accSum[0] += accADC[0];
    accSum[1] += accADC[1];
    accSum[2] += accADC[2];
    DelayUs(2500);
  }

  accZero[0] = accSum[0] / 100;
  accZero[1] = accSum[1] / 100;
  accZero[2] = accSum[2] / 100 - (int) (1.0 / (float) ACC_SCALE);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void ADXL345_Update(float * accValue) {

  ADXL345_ReadADC();

  accValue[0] = (accADC[0] - accZero[0]) * ACC_SCALE;
  accValue[1] = (accADC[1] - accZero[1]) * ACC_SCALE;
  accValue[2] = (accADC[2] - accZero[2]) * ACC_SCALE;
}

