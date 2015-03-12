/*
 * HMC5883.c
 *
 * I2C Compass HMC5883
 *
 * I2C adress: 0x3C (8bit)   0x1E (7bit)
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#include <inc/hw_types.h>
#include <driverlib/i2c.h>
#include <math.h>
#include "../utils/delay.h"
#include "../utils/linalg.h"
#include "../utils/i2c_functions.h"
#include "HMC5883.h"


extern uint8_t  rawADC[6];

int     magADC[3];
float   magGain[3] = { 1.0, 1.0, 1.0 }; // gain for each axis, populated at sensor init
int   magZero[3];

//*****************************************************************************
//
// return 1 when news values are available, 0 otherwise
//
//*****************************************************************************
void HMC5883_GetRawADC() {
  I2CReadMultiRegisters(MAG_ADDRESS, MAG_DATA_REGISTER, rawADC, 6);

  MAG_ORIENTATION( ((int16_t)(rawADC[0]<<8) | (int16_t)rawADC[1]),
                   ((int16_t)(rawADC[4]<<8) | (int16_t)rawADC[5]),
                   ((int16_t)(rawADC[2]<<8) | (int16_t)rawADC[3]));
}

//*****************************************************************************
//
// return 1 when news values are available, 0 otherwise
//
//*****************************************************************************
uint8_t HMC5883_Update(float * magValue) {
  static uint32_t deadline = 0;
  uint64_t currentTime = micros();

  if(deadline == 0)
    deadline = currentTime-1;

  if (currentTime < deadline)
    return 0; //each read is spaced by 100ms
  deadline += 67000;

  HMC5883_GetRawADC();

  magValue[0] = (magADC[0] - magZero[0]) * magGain[0];
  magValue[1] = (magADC[1] - magZero[1]) * magGain[1];
  magValue[2] = (magADC[2] - magZero[2]) * magGain[2];

  return 1;
}


//*****************************************************************************
//
//
//
//*****************************************************************************
void HMC5883_SetCalibration(int * cal) {
  magZero[0] = cal[0];
  magZero[1] = cal[1];
  magZero[2] = cal[2];
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void HMC5883_GetCalibration(int * zero) {
  zero[0] = magZero[0];
  zero[1] = magZero[1];
  zero[2] = magZero[2];
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void HMC5883_Calibrate() {
  int i, axis;
  int min[3], max[3];

  min[0] = 2048;
  min[1] = 2048;
  min[2] = 2048;
  max[0] = -2048;
  max[1] = -2048;
  max[2] = -2048;

  for (i = 0; i < 100; i++) {
    HMC5883_GetRawADC();
    for (axis = 0; axis < 3; axis++) {
      if(magADC[axis] < min[axis])
        min[axis] = magADC[axis];
      if(magADC[axis] > max[axis])
        max[axis] = magADC[axis];
    }
    DelayMs(67);
  }

  magZero[0] = (max[0] + min[0]) / 2;
  magZero[1] = (max[1] + min[1]) / 2;
  magZero[2] = (max[2] + min[2]) / 2;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void HMC5883_Init() {
  int i;
  int xyz_total[3] = { 0, 0, 0 };  // 32 bit totals so they won't overflow.
  tBoolean bret = true;                // Error indicator

  DelayMs(50);  //Wait before start
  I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
  // The new gain setting is effective from the second measurement and on.

  I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_MODE, 1);
  DelayMs(100);
  HMC5883_GetRawADC();  //Get one sample, and discard it

  for (i = 0; i < 10; i++) { //Collect 10 samples
    I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_MODE, 1);
    DelayMs(100);
    HMC5883_GetRawADC(); // Get the raw values in case the scales have already been changed.

    // Since the measurements are noisy, they should be averaged rather than taking the max.
    xyz_total[0] += magADC[0];
    xyz_total[1] += magADC[1];
    xyz_total[2] += magADC[2];

    // Detect saturation.
    if (-(1 << 12) >= min(magADC[0], min(magADC[1], magADC[2]))) {
      bret = false;
      break; // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

// Apply the negative bias. (Same gain)
  I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
  for (i = 0; i < 10; i++) {
    I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_MODE, 1);
    DelayMs(100);
    HMC5883_GetRawADC(); // Get the raw values in case the scales have already been changed.

    // Since the measurements are noisy, they should be averaged.
    xyz_total[0] -= magADC[0];
    xyz_total[1] -= magADC[1];
    xyz_total[2] -= magADC[2];

    // Detect saturation.
    if (-(1 << 12) >= min(magADC[0], min(magADC[1], magADC[2]))) {
      bret = false;
      break; // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  magGain[0] = fabs(
      820.0 * HMC58X3_X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[0]);
  magGain[1] = fabs(
      820.0 * HMC58X3_Y_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[1]);
  magGain[2] = fabs(
      820.0 * HMC58X3_Z_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[2]);

  // leave test mode
  I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_CONFA, 0x70); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  I2CWriteRegister(MAG_ADDRESS, HMC58X3_R_MODE, 0x00); //Mode register             -- 000000 00    continuous Conversion Mode
  DelayMs(100);

  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }

}
