/*
 * sensors.c
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#include <math.h>
#include <inc/hw_types.h>
#include <driverlib/i2c.h>
#include "utils/delay.h"
#include "utils/linalg.h"
#include "sensors/ADXL345.h"
#include "sensors/BMP085.h"
#include "sensors/HMC5883.h"
#include "sensors/L3G4200D.h"
#include "sensors.h"


/************ Global variables *************/
uint8_t   rawADC[6];
float     pressure, temperature;
float     gyroValue[3], accValue[3], magValue[3];
float     gyroVar, accVar;
State     state[2];

float     gff[3][3], aff[3][3];
float     gfb[3][3], afb[3][3];


//*****************************************************************************
//
// Sensors init
//
//*****************************************************************************
void SensorsInit(int flags) {
  L3G4200D_Init();
  ADXL345_Init();
  BMP085_Init();
  HMC5883_Init();

  if (flags & ACC_CALIBRATE)
    accVar  = ADXL345_Calibrate();
  if (flags & GYRO_CALIBRATE)
    gyroVar = L3G4200D_Calibrate();
  if (flags & MAG_CALIBRATE)
    HMC5883_Calibrate();

  KalmanInit();
}


//*****************************************************************************
//
// Sensors Low Pass Filter
//
// 50 Hz:
// b = [ 0.0902,  0.1804,  0.0902]
// a = [ 1.0000, -0.9895,  0.3503]
//
// 25 Hz:
// b = [ 0.0293,  0.0586,  0.0293]
// a = [ 1.0000, -1.4609,  0.5781]
//
//*****************************************************************************
void SensorsLowPassFilter() {
  int i, j;
  // swift previous data
  for (i = 0; i < 3; i++) {
    for (j = 1; j < 3; j++) {
      gff[i][j] = gff[i][j-1];
      gfb[i][j] = gfb[i][j-1];
      aff[i][j] = aff[i][j-1];
      afb[i][j] = afb[i][j-1];
    }
  }
  // apply 25Hz low pass filter
  for (i = 0; i < 3; i++) {
    gff[i][0] = gyroValue[i];
    gfb[i][0] = 0.0293*gff[i][0] + 0.0586*gff[i][1] + 0.0293*gff[i][2]
                                 + 1.4609*gfb[i][1] - 0.5781*gfb[i][2];
    gyroValue[i] = gfb[i][0];
    aff[i][0] = accValue[i];
    afb[i][0] = 0.0293*aff[i][0] + 0.0586*aff[i][1] + 0.0293*aff[i][2]
                                 + 1.4609*afb[i][1] - 0.5781*afb[i][2];
    accValue[i] = afb[i][0];

  }
}

//*****************************************************************************
//
// Sensors update
//
//*****************************************************************************
void SensorsUpdate() {
  L3G4200D_Update(gyroValue);   // 400Hz
  ADXL345_Update(accValue);    // 400Hz
  HMC5883_Update(magValue);    // 15Hz
  BMP085_Update(&pressure, &temperature);     // ~40Hz

  SensorsLowPassFilter();
}

//*****************************************************************************
//
// Set sensors calibration values
//
//*****************************************************************************
void SensorsSetCalibration(int * accZero, int * magZero) {
  ADXL345_SetCalibration(accZero);
  HMC5883_SetCalibration(magZero);
}

//*****************************************************************************
//
// Get sensors calibration values
//
//*****************************************************************************
void SensorsGetCalibration(int * accZero, int * magZero) {
  ADXL345_GetCalibration(accZero);
  HMC5883_GetCalibration(magZero);
}

//*****************************************************************************
//
// Kalman filter init
//
//*****************************************************************************
void KalmanInit() {
  // Process noise variance
  const float PNstd = 0.05;
  const float PNV = PNstd*PNstd;
  // Measurement noise variance
  const float accMNV = ACC_SCALE*ACC_SCALE*100;
  const float gyroMNV = GYRO_SCALE*GYRO_SCALE*10;

  state[ROLL].x.angle = 0;
  state[ROLL].x.rate  = 0;
  state[ROLL].dt      = SAMPLE_TIME;
  state[ROLL].Q_angle = PNV;
  state[ROLL].Q_rate  = PNV;
  state[ROLL].R_angle = accMNV;
  state[ROLL].R_rate  = gyroMNV;
  state[ROLL].P[0]    = accMNV;
  state[ROLL].P[1]    = 0;
  state[ROLL].P[2]    = 0;
  state[ROLL].P[3]    = gyroMNV;

  state[PITCH].x.angle = 0;
  state[PITCH].x.rate  = 0;
  state[PITCH].dt      = SAMPLE_TIME;
  state[PITCH].Q_angle = PNV;
  state[PITCH].Q_rate  = PNV;
  state[PITCH].R_angle = accMNV;
  state[PITCH].R_rate  = gyroMNV;
  state[PITCH].P[0]    = accMNV;
  state[PITCH].P[1]    = 0;
  state[PITCH].P[2]    = 0;
  state[PITCH].P[3]    = gyroMNV;
}

//*****************************************************************************
//
// Kalman filter
//
//*****************************************************************************
void KalmanFilter(State* s) {
  float Y[4], iY[4], E[2];

  // Prediction for state vector and covariance
  // xp = Ax + Bu
  s->x.angle += s->dt * s->x.rate;
  // Pp = APA' + Q
  s->P[0] += s->Q_angle + s->dt * (s->P[1] + s->P[2] + s->dt * s->P[3]);
  s->P[1] += s->dt * s->P[3];
  s->P[2] += s->dt * s->P[3];
  s->P[3] += s->Q_rate;

  // Compute Kalman gain factor:
  // K = PpH'inv(HPpH' + R)
  Y[0] = s->R_angle + s->P[0];
  Y[1] = s->P[1];
  Y[2] = s->P[2];
  Y[3] = s->R_rate + s->P[3];
  Inv2(Y, iY);
  Dot2(s->P, iY, s->K);

  // Correction based on observation:
  // x = xp + K(z - Hxp)
  E[0] = (s->z.angle - s->x.angle);
  E[1] = (s->z.rate - s->x.rate);
  s->x.angle += s->K[0] * E[0] + s->K[1] * E[1];
  s->x.rate  += s->K[2] * E[0] + s->K[3] * E[1];
  // P = Pp - KHPp
  Dot2(s->K, s->P, Y);
  s->P[0] -= Y[0];
  s->P[1] -= Y[1];
  s->P[2] -= Y[2];
  s->P[3] -= Y[3];
}

//*****************************************************************************
//
// Estimate the real attitude based
//
//*****************************************************************************
void AttitudeEstimation(float * roll, float * rollRate, float * pitch, float * pitchRate, float * yaw, float * altitude) {
  // calculate observed angles and rates
  state[ROLL].z.angle  = rad2deg(atan2(accValue[ROLL] , accValue[YAW]));
  state[ROLL].z.rate   = gyroValue[ROLL];
  state[PITCH].z.angle = rad2deg(atan2(accValue[PITCH], accValue[YAW]));
  state[PITCH].z.rate  = gyroValue[PITCH];

  // compute Kalman filter
  KalmanFilter(&state[ROLL] );
  KalmanFilter(&state[PITCH]);

  // get predicted angles and rates
  *roll  = state[ROLL].x.angle;
  *rollRate = state[ROLL].x.rate;
  *pitch = state[PITCH].x.angle;
  *pitchRate = state[PITCH].x.rate;


  // calculate the heading
  *yaw = rad2deg(atan2(magValue[1], magValue[0]));
  if(*yaw < 0)
    *yaw += 360;

  // calculate the altitude
  *altitude = 44307.694 * (1.0 - pow((double)pressure / DEFAULT_PRESSURE, 0.190284));
}



