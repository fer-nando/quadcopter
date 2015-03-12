/*
 * L3G4200D.c
 *
 * I2C Gyroscope L3G4200D
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#include <inc/hw_types.h>
#include <driverlib/i2c.h>
#include "../utils/delay.h"
#include "../utils/linalg.h"
#include "../utils/i2c_functions.h"
#include "L3G4200D.h"


extern uint8_t rawADC[6];

int   gyroADC[3];
int   gyroZero[3] = { 0, 0, 0 };


//*****************************************************************************
//
//
//
//*****************************************************************************
void L3G4200D_Init() {
  DelayMs(100);
  I2CWriteRegister(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1, 0x8F); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  DelayMs(5);
  I2CWriteRegister(L3G4200D_ADDRESS, L3G4200D_CTRL_REG5, 0x02); // CTRL_REG5   low pass filter enable
  DelayMs(5);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void L3G4200D_ReadADC() {
  I2CReadMultiRegisters(L3G4200D_ADDRESS, 0x80 | L3G4200D_OUT_X_L, rawADC, 6);

  GYRO_ORIENTATION( ((int16_t)(rawADC[1]<<8) | (int16_t)rawADC[0]),
                    ((int16_t)(rawADC[3]<<8) | (int16_t)rawADC[2]),
                    ((int16_t)(rawADC[5]<<8) | (int16_t)rawADC[4]));
}

//*****************************************************************************
//
//
//
//*****************************************************************************
float L3G4200D_Calibrate() {
  int i;
  int gyroSum[3] = {0,0,0};
  float gyroVar;
  float gyroValue[3];
  float gyroVarSum[3] = { 0, 0, 0 };

  for (i = 0; i < 100; i++) {
    L3G4200D_ReadADC();
    gyroSum[0] += gyroADC[0];
    gyroSum[1] += gyroADC[1];
    gyroSum[2] += gyroADC[2];
    DelayUs(2500);
  }

  gyroZero[0] = gyroSum[0] / 100;
  gyroZero[1] = gyroSum[1] / 100;
  gyroZero[2] = gyroSum[2] / 100;

  for (i = 0; i < 100; i++) {
    L3G4200D_Update(gyroValue);
    gyroVarSum[0] += (gyroValue[0]*gyroValue[0]);
    gyroVarSum[1] += (gyroValue[1]*gyroValue[1]);
    gyroVarSum[2] += (gyroValue[2]*gyroValue[2]);
    DelayUs(2500);
  }

  gyroVar = (gyroVarSum[0] / 100) + (gyroVarSum[1] / 100) + (gyroVarSum[2] / 100) / 3.0;
  return gyroVar;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void L3G4200D_Update(float * gyroValue) {
  int axis;
  static int previousGyroADC[3] = { 0, 0, 0 };

  L3G4200D_ReadADC();

  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis] -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    gyroADC[axis] = constrain(gyroADC[axis], (previousGyroADC[axis]-800), (previousGyroADC[axis]+800));
    previousGyroADC[axis] = gyroADC[axis];
    gyroValue[axis] = gyroADC[axis] * GYRO_SCALE;
  }
}
