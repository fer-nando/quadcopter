/*
 * motor.c
 *
 *  Created on: 04/02/2015
 *      Author: Fernando
 */

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "motor.h"

void MotorInit(void) {

  // Enable peripherals used.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

  // Configure the GPIO pin muxing for the Timer/CCP function.
  GPIOPinConfigure(GPIO_PD0_WT2CCP0);
  GPIOPinConfigure(GPIO_PD1_WT2CCP1);
  GPIOPinConfigure(GPIO_PD2_WT3CCP0);
  GPIOPinConfigure(GPIO_PD3_WT3CCP1);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_1);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_2);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_3);

  // Configure Wide Timers 2 and 3 as two 32-bit PWM timers.
  TimerConfigure(WTIMER2_BASE,
      TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
  TimerConfigure(WTIMER3_BASE,
      TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
}

void MotorEnable() {
  // Enable both timers of WTIMER2 and WTIMER3.
  TimerEnable(WTIMER2_BASE, TIMER_BOTH);
  TimerEnable(WTIMER3_BASE, TIMER_BOTH);
}

void MotorDisable() {
  // Disable both timers of WTIMER2 and WTIMER3.
  TimerDisable(WTIMER2_BASE, TIMER_BOTH);
  TimerDisable(WTIMER3_BASE, TIMER_BOTH);
}

void MotorSetFrequency(int frequency) {
  unsigned long loadValue = (unsigned long) (SysCtlClockGet() / frequency) - 1;

  TimerLoadSet(WTIMER2_BASE, TIMER_A, loadValue);
  TimerLoadSet(WTIMER2_BASE, TIMER_B, loadValue);
  TimerLoadSet(WTIMER3_BASE, TIMER_A, loadValue);
  TimerLoadSet(WTIMER3_BASE, TIMER_B, loadValue);
}

void MotorSetDutyCycle(float dutyCycle[4]) {
  unsigned long matchValue;
  unsigned long loadValue = TimerLoadGet(WTIMER2_BASE, TIMER_A);
  unsigned long milliseconds = SysCtlClockGet() / 1000;

  matchValue = loadValue - (unsigned long) ((dutyCycle[0] / 100.0 + 1.0) * milliseconds);
  TimerMatchSet(WTIMER2_BASE, TIMER_A, matchValue);
  matchValue = loadValue - (unsigned long) ((dutyCycle[1] / 100.0 + 1.0) * milliseconds);
  TimerMatchSet(WTIMER2_BASE, TIMER_B, matchValue);
  matchValue = loadValue - (unsigned long) ((dutyCycle[2] / 100.0 + 1.0) * milliseconds);
  TimerMatchSet(WTIMER3_BASE, TIMER_A, matchValue);
  matchValue = loadValue - (unsigned long) ((dutyCycle[3] / 100.0 + 1.0) * milliseconds);
  TimerMatchSet(WTIMER3_BASE, TIMER_B, matchValue);
}

void MotorSetMicroseconds(int dutyCycle) {
  unsigned long loadValue = TimerLoadGet(WTIMER2_BASE, TIMER_A);
  unsigned long matchValue = loadValue - (dutyCycle *  (SysCtlClockGet() / 1000000));

  TimerMatchSet(WTIMER2_BASE, TIMER_A, matchValue);
  TimerMatchSet(WTIMER2_BASE, TIMER_B, matchValue);
  TimerMatchSet(WTIMER3_BASE, TIMER_A, matchValue);
  TimerMatchSet(WTIMER3_BASE, TIMER_B, matchValue);
}
