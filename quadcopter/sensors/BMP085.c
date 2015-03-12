/*
 * BMP085.c
 *
 * I2C Barometer BOSCH BMP085
 *
 * I2C adress: 0x77 (7bit)
 * principle:
 *  1) read the calibration register (only once at the initialization)
 *  2) read uncompensated temperature (not mandatory at every cycle)
 *  3) read uncompensated pressure
 *  4) raw temp + raw pressure => calculation of the adjusted pressure
 *  the following code uses the maximum precision setting (oversampling setting 3)
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#include <math.h>
#include <inc/hw_types.h>
#include <driverlib/i2c.h>
#include "../utils/delay.h"
#include "../utils/linalg.h"
#include "../utils/i2c_functions.h"
#include "BMP085.h"


int    baroPressure;
int    baroTemperature;
int    baroPressureSum;
BMP085 bmp085_ctx;

//*****************************************************************************
//
//
//
//*****************************************************************************
void BMP085_ReadCalibration() {
  DelayMs(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*) &bmp085_ctx.md - (uint8_t*) &bmp085_ctx.ac1
      + sizeof(bmp085_ctx.ac1);
  I2CReadMultiRegisters(BMP085_ADDRESS, 0xAA, (uint8_t*) &bmp085_ctx.ac1,
      s_bytes);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    SwapEndianness((uint8_t*) p, 2);
  }
}

//*****************************************************************************
//
// read uncompensated temperature value: send command first
//
//*****************************************************************************
void BMP085_UT_Start() {
  I2CWriteRegister(BMP085_ADDRESS, 0xf4, 0x2e);
  I2CWrite(0xF6, I2C_MASTER_CMD_SINGLE_SEND);
}

//*****************************************************************************
//
// read uncompensated pressure value: send command first
//
//*****************************************************************************
void BMP085_UP_Start() {
  I2CWriteRegister(BMP085_ADDRESS, 0xf4, 0x34 + (OSS << 6)); // control register value for oversampling setting 3
  I2CWrite(0xF6, I2C_MASTER_CMD_SINGLE_SEND);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void BMP085_Init() {
  DelayMs(10);
  BMP085_ReadCalibration();
  DelayMs(5);
  BMP085_UT_Start();
  bmp085_ctx.deadline = micros() + 5000;
}

//*****************************************************************************
//
// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
//
//*****************************************************************************
void BMP085_UP_Read() {
  I2CReadMulti(BMP085_ADDRESS, bmp085_ctx.up.raw, 3);
  SwapEndianness(bmp085_ctx.up.raw, 3);
}

//*****************************************************************************
//
// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
//
//*****************************************************************************
void BMP085_UT_Read() {
  I2CReadMulti(BMP085_ADDRESS, bmp085_ctx.ut.raw, 2);
  SwapEndianness(bmp085_ctx.ut.raw, 2);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void BMP085_Calculate() {
  int32_t x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;

  // Temperature calculations
  x1 = ((int32_t) bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t) bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)

  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11;
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp * 4 + x3) << OSS;
  b3 = (tmp + 2) >> 2;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8 - OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  baroPressure = p + ((x1 + x2 + 3791) >> 4);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void BMP085_MeanFilter() {
  static int baroHistTab[BARO_TAB_SIZE];
  static int first;

  uint8_t last = (first + 1);
  if (last == BARO_TAB_SIZE)
    last = 0;
  baroHistTab[first] = baroPressure;
  baroPressureSum += baroHistTab[first];
  baroPressureSum -= baroHistTab[last];
  first = last;
}

//*****************************************************************************
//
// return 0: no data available, no computation;
//        1: new value available;
//        2: no new value, but computation time.
//
//*****************************************************************************
uint8_t BMP085_Update(float * pressure, float * temperature) {    // first UT conversion is started in init procedure
  uint64_t currentTime = micros();
  if (currentTime < bmp085_ctx.deadline)
    return 0;
  if (bmp085_ctx.state == 0) {
    BMP085_UT_Read();
    BMP085_UP_Start();
    bmp085_ctx.state = 1;
    BMP085_MeanFilter();
    *pressure = (float) baroPressureSum / BARO_TAB_SIZE;
    bmp085_ctx.deadline += 21000; // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)
    return 1;
  } else {
    BMP085_UP_Read();
    BMP085_UT_Start();
    BMP085_Calculate();
    *temperature = (float) baroTemperature / 100.0;
    bmp085_ctx.state = 0;
    bmp085_ctx.deadline = currentTime + 6000; // 1.5ms margin according to the spec (4.5ms T conversion time)
    return 2;
  }
}
