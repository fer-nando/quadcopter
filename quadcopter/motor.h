/*
 * motor.h
 *
 *  Created on: 04/02/2015
 *      Author: Fernando
 */

#ifndef MOTOR_H_
#define MOTOR_H_


void MotorInit(void);
void MotorEnable(void);
void MotorDisable(void);
void MotorSetFrequency(int frequency);
void MotorSetDutyCycle(float dutyCycle[4]);
void MotorSetMicroseconds(int dutyCycle);


#endif /* MOTOR_H_ */
