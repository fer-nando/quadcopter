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
State     state[2];

#ifdef USE_LPF
float     gff[3][3], aff[3][3];
float     gfb[3][3], afb[3][3];
#endif


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
    ADXL345_Calibrate();
  if (flags & GYRO_CALIBRATE)
    L3G4200D_Calibrate();
  if (flags & MAG_CALIBRATE)
    HMC5883_Calibrate();


  KalmanInit();
}


//*****************************************************************************
//
// Sensors Low Pass Filter
//
//
//*****************************************************************************
#ifdef USE_LPF
void SensorsLowPassFilter() {
  int i, j;
  // swift previous data
  for (i = 0; i < 3; i++) {
    for (j = 2; j > 0; j--) {
      gff[i][j] = gff[i][j-1];
      gfb[i][j] = gfb[i][j-1];
      aff[i][j] = aff[i][j-1];
      afb[i][j] = afb[i][j-1];
    }
  }
  // apply 25Hz low pass filter
  for (i = 0; i < 3; i++) {

    gff[i][0] = gyroValue[i];
    aff[i][0] = accValue[i];

#ifdef ACC_LPF_50HZ
    afb[i][0] = 0.09021*aff[i][0] + 0.18041*aff[i][1] + 0.09021*aff[i][2]
                                  + 0.98947*afb[i][1] - 0.35029*afb[i][2];
#else  // ACC_LPF != 50HZ
#ifdef ACC_LPF_25HZ
    afb[i][0] = 0.02929*aff[i][0] + 0.05858*aff[i][1] + 0.02929*aff[i][2]
                                  + 1.46091*afb[i][1] - 0.57807*afb[i][2];
#else  // ACC_LPF != 25Hz
#ifdef ACC_LPF_15HZ
    afb[i][0] = 0.01176*aff[i][0] + 0.02351*aff[i][1] + 0.01176*aff[i][2]
                                  + 1.67070*afb[i][1] - 0.71773*afb[i][2];
#else // ACC_LPF != 15HZ
#ifdef ACC_LPF_10HZ
    afb[i][0] = 0.00552*aff[i][0] + 0.01104*aff[i][1] + 0.00552*aff[i][2]
                                  + 1.77908*afb[i][1] - 0.80117*afb[i][2];
#else // ACC_LPF != 10HZ
#ifdef ACC_LPF_5HZ
    afb[i][0] = 0.00146*aff[i][0] + 0.00292*aff[i][1] + 0.00146*aff[i][2]
                                  + 1.88909*afb[i][1] - 0.89493*afb[i][2];
#endif // ACC_LPF_5HZ
#endif // ACC_LPF_10HZ
#endif // ACC_LPF_15HZ
#endif // ACC_LPF_25HZ
#endif // ACC_LPF_50HZ

#ifdef GYRO_LPF_100HZ
    gfb[i][0] = 0.22615*gff[i][0] + 0.45231*gff[i][1] + 0.22615*gff[i][2]
                                  + 0.28095*gfb[i][1] - 0.18556*gfb[i][2];
#else // GYRO_LPF != 100HZ
#ifdef GYRO_LPF_50HZ
    gfb[i][0] = 0.09021*gff[i][0] + 0.18041*gff[i][1] + 0.09021*gff[i][2]
                                  + 0.98947*gfb[i][1] - 0.35029*gfb[i][2];
#else // GYRO_LPF != 50HZ
#ifdef GYRO_LPF_25HZ
    gfb[i][0] = 0.02929*gff[i][0] + 0.05858*gff[i][1] + 0.02929*gff[i][2]
                                  + 1.46091*gfb[i][1] - 0.57807*gfb[i][2];
#endif // GYRO_LPF_25HZ
#endif // GYRO_LPF_50HZ
#endif // GYRO_LPF_100HZ

    gyroValue[i] = gfb[i][0];
    accValue[i] = afb[i][0];

  }
}
#endif

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
#ifdef USE_LPF
  SensorsLowPassFilter();
#endif
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
  state[ROLL].x.angle = 0;
  state[ROLL].x.rate  = 0;
  state[ROLL].dt      = SAMPLE_TIME;
  state[ROLL].Q_angle = PROCESS_VAR;
  state[ROLL].Q_rate  = PROCESS_VAR;
  state[ROLL].R_angle = ACC_VAR;
  state[ROLL].R_rate  = GYRO_VAR;
  state[ROLL].P[0]    = 1;
  state[ROLL].P[1]    = 0;
  state[ROLL].P[2]    = 0;
  state[ROLL].P[3]    = 1;

  state[PITCH].x.angle = 0;
  state[PITCH].x.rate  = 0;
  state[PITCH].dt      = SAMPLE_TIME;
  state[PITCH].Q_angle = PROCESS_VAR;
  state[PITCH].Q_rate  = PROCESS_VAR;
  state[PITCH].R_angle = ACC_VAR;
  state[PITCH].R_rate  = GYRO_VAR;
  state[PITCH].P[0]    = 1;
  state[PITCH].P[1]    = 0;
  state[PITCH].P[2]    = 0;
  state[PITCH].P[3]    = 1;
}

//*****************************************************************************
//
// Kalman filter
//
//*****************************************************************************
#ifdef KALMAN_FILTER
void KalmanFilter(State* s) {
  float Y[4], invY[4], E[2];

  // Prediction for state vector and covariance
  // xp = Ax + Bu
  s->x.angle += (s->dt * s->x.rate);
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
  Inv2(Y, invY);
  Dot2(s->P, invY, s->K);

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
#endif

//*****************************************************************************
//
// Complementary filter
//
//*****************************************************************************
#ifdef COMPLEMENTARY_FILTER
void ComplementaryFilter(State* s) {
  //const float a = TAU/(TAU+SAMPLE_TIME);
  const float a = 0.995;
  s->x.angle = a*(s->x.angle + s->z.rate*s->dt) + (1-a)*(s->z.angle);
  s->x.rate  = s->z.rate;
}
#endif

//*****************************************************************************
//
// Estimate the real attitude based
//
//*****************************************************************************
void AttitudeEstimation(float * pitch, float * pitchRate, float * roll, float * rollRate, float * yaw, float * yawRate, float * altitude) {
  // calculate observed angles and rates
  state[0].z.angle  = rad2deg(atan2(accValue[0] , accValue[2]));
  state[0].z.rate   = gyroValue[0];
  state[1].z.angle = rad2deg(atan2(accValue[1], accValue[2]));
  state[1].z.rate  = gyroValue[1];

#ifdef KALMAN_FILTER
  // compute Kalman filter
  KalmanFilter(&state[0]);
  KalmanFilter(&state[1]);
#endif
#ifdef COMPLEMENTARY_FILTER
  // compute Complementary filter
  ComplementaryFilter(&state[0]);
  ComplementaryFilter(&state[1]);
#endif

  // get predicted angles and rates
  *pitch = state[0].x.angle;
  *pitchRate = state[0].z.rate;
  *roll  = state[1].x.angle;
  *rollRate = state[1].z.rate;

  // calculate the heading
  *yaw = rad2deg(atan2(magValue[1], magValue[0]));
  *yawRate = gyroValue[2];
  //if(*yaw < 0)
  //  *yaw += 360;

  // calculate the altitude
  *altitude = 44307.694 * (1.0 - pow((double)pressure / DEFAULT_PRESSURE, 0.190284));
}



