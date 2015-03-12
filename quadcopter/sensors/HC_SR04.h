/*
 * HC_SR04.h
 *
 *  Created on: 05/02/2015
 *      Author: Fernando
 */

#ifndef HC_SR04_H_
#define HC_SR04_H_


// parameters
#define SONAR_HIST_SIZE  10
#define SONAR_DEADLINE   29412         // time to sound travel 5m (in microseconds)
#define SONAR_SCALE      0.017241379   // = (speed of sound) / 2 / (us to s) * (m to cm)
                                      // = 340 / 2 / 1.000.000 / 100

// functions
void HC_SR04_Init();
void HC_SR04_Update();
float HC_SR04_GetDistance();
int HC_SR04_GetOORCount();


#endif /* HC_SR04_H_ */
