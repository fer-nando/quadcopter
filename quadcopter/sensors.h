#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdint.h>

// RC alias
enum rc {
  ROLL, PITCH, YAW, THROTTLE
};
#define RC_CHANS                    4

// IMU Orientations and Sensor definitions
#define CALIBRATION_LENGHT          100           // num samples for calibration
#define SAMPLE_TIME                 0.0025        // 2.5 ms
#define SAMPLE_TIME_US              2500          // 2,500 us
#define SAMPLE_RATE                 400           // 400 Hz
#define GYRO_CALIBRATE              1
#define ACC_CALIBRATE               2
#define MAG_CALIBRATE               4

//#define USE_LPF
  #define LPF_50HZ
  //#define LPF_25HZ

//#define KALMAN_FILTER
#define COMPLEMENTARY_FILTER

// structures
typedef struct Atitude {
  float angle, rate;
} Atitude;

typedef struct State {
  Atitude x;      // state vector
  Atitude z;      // observation vector
  //float u;      // control vector

  //float A[4];   // state transition matrix -- dynamics
  float dt;
  //float B;      // input matrix (maps control commands onto state changes)
  float P[4];     // covariance of state vector estimate
  //float Q[4];   // process noise covariance
  float Q_angle, Q_rate;
  //float R[4];   // measurement noise covariance
  float R_angle, R_rate;
  //float H[4];   // observation matrix
  float K[4];     // kalman gain factor
} State;

// functions
void SensorsInit(int flags);
void SensorsUpdate();
void SensorsSetCalibration(int * accZero, int * magZero);
void SensorsGetCalibration(int * accZero, int * magZero);
void KalmanInit();
void KalmanFilter(State* s);
void ComplementaryFilter(State* s);
void AttitudeEstimation(float * roll, float * rollRate, float * pitch, float * pitchRate, float * yaw, float * altitude);

#endif
