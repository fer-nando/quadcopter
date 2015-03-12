/*
 * HMC5883.h
 *
 * I2C Compass HMC5883
 *
 *  Created on: 13/12/2014
 *      Author: Fernando1
 */

#ifndef HMC5883_H_
#define HMC5883_H_

// registers
#define MAG_DATA_REGISTER           0x03

// device address
#define MAG_ADDRESS 0x1E

// parameters
#define MAG_ORIENTATION(X, Y, Z)    {magADC[0] = X; magADC[1] = Y; magADC[2] = -Z;}
#define HMC58X3_R_CONFA             0
#define HMC58X3_R_CONFB             1
#define HMC58X3_R_MODE              2
#define HMC58X3_X_SELF_TEST_GAUSS   (+1.16)   //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS   (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS   (+1.08)   //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT         (243.0/390.0)  //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT        (575.0/390.0)  //!< High limit when gain is 5.
#define HMC_POS_BIAS                1
#define HMC_NEG_BIAS                2

// functions
void HMC5883_GetRawADC();
void HMC5883_Init();
void HMC5883_SetCalibration(int * cal);
void HMC5883_GetCalibration(int * zero);
void HMC5883_Calibrate();
uint8_t HMC5883_Update(float * magValue);


#endif /* HMC5883_H_ */
