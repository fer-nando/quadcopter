/*
 * BMP085.h
 *
 * I2C Barometer BOSCH BMP085
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#ifndef BMP085_H_
#define BMP085_H_

// device address
#define BMP085_ADDRESS    0x77

// parameters
#define BARO_TAB_SIZE     21
#define OSS               3
#define DEFAULT_PRESSURE  101325.0

typedef struct BMP085 {
  // sensor registers from the BOSCH BMP085 datasheet
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2, mb, mc, md;
  union {
    uint16_t val;
    uint8_t raw[2];
  } ut; //uncompensated temperature
  union {
    uint32_t val;
    uint8_t raw[4];
  } up; //uncompensated pressure
  uint8_t state;
  uint32_t deadline;
} BMP085;

// functions
void BMP085_ReadCalibration();
void BMP085_UT_Start();
void BMP085_UP_Start();
void BMP085_Init();
void BMP085_UP_Read();
void BMP085_UT_Read();
void BMP085_Calculate();
void BMP085_MeanFilter();
uint8_t BMP085_Update(float * pressure, float * temperature);

#endif /* BMP085_H_ */
