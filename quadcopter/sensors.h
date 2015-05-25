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

// Low-pass filter
#define USE_LPF
  //#define ACC_LPF_50HZ
  //#define ACC_LPF_25HZ
  //#define ACC_LPF_15HZ
  //#define ACC_LPF_10HZ
  #define ACC_LPF_5HZ
  //#define GYRO_LPF_100HZ
  #define GYRO_LPF_50HZ
  //#define GYRO_LPF_25HZ

// Kalman filter
#define KALMAN_FILTER
  #define ACC_STD       40
  #define GYRO_STD      5.38
  #define PROCESS_STD   0.20

  #define ACC_VAR       (ACC_STD*ACC_STD)
  #define GYRO_VAR      (GYRO_STD*GYRO_STD)
  #define PROCESS_VAR   (PROCESS_STD*PROCESS_STD)

// Complementary filter
//#define COMPLEMENTARY_FILTER
  #define TAU           0.75


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
void AttitudeEstimation(float * roll, float * rollRate, float * pitch, float * pitchRate, float * yaw, float * yawRate, float * altitude);

#endif
