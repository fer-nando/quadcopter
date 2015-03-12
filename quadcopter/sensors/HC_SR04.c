/*
 * HC_SR04.c
 *
 *  Created on: 05/02/2015
 *      Author: Fernando
 */

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "../utils/delay.h"
#include "HC_SR04.h"


float sonarDistanceSum = 0;
int sonarOORCount = 0;

volatile tBoolean sonarReady;
volatile float sonarDistance;
volatile long sonarEchoDur;
volatile unsigned long sonarEchoStart;


void HC_SR04_Init() {

  // Enable the peripherals used.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

  // Configure the GPIO PB0 pin muxing for the Timer/CCP function.
  GPIOPinConfigure(GPIO_PB0_T2CCP0);
  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);

  // Configure Timers 2 as 16-bit PWM timer.
  TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
  TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);

  unsigned long loadValue = (unsigned long) SONAR_DEADLINE * (SysCtlClockGet() / 1000000) - 1; // deadline * clock / 1.000.000 = 80*deadline
  unsigned long matchValue = loadValue - (SysCtlClockGet() / 100000); // 10 us = clock / 100.000
  TimerLoadSet(TIMER2_BASE, TIMER_A, loadValue & 0x0000ffff);
  TimerPrescaleSet(TIMER2_BASE, TIMER_A, (loadValue & 0x00ff0000) >> 16);
  TimerMatchSet(TIMER2_BASE, TIMER_A, matchValue & 0x0000ffff);
  TimerPrescaleMatchSet(TIMER2_BASE, TIMER_A, (matchValue & 0x00ff0000) >> 16);

  // Configure the PB1 to be the input.
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_1);
  GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_BOTH_EDGES);
  GPIOPinIntEnable(GPIO_PORTB_BASE, GPIO_PIN_1);

  // Enable the interrupts.
  IntEnable(INT_GPIOB);
  IntEnable(INT_TIMER2A);

  // Enable timer TIMER 2.
  TimerEnable(TIMER2_BASE, TIMER_A);

  sonarReady = false;
}


void HC_SR04_Update() {
  static float sonarHist[SONAR_HIST_SIZE];
  static int first;

  if(sonarReady) {
    sonarReady = false;

    if (sonarDistance >= 0) {
      sonarOORCount = 0;
      int last = first + 1;
      if (last == SONAR_HIST_SIZE)
        last = 0;

      sonarHist[first] = sonarDistance;
      sonarDistanceSum += sonarDistance;
      sonarDistanceSum -= sonarHist[last];
      first = last;
    } else {
      sonarOORCount++;
    }
  }

}

float HC_SR04_GetDistance() {
  return sonarDistanceSum / SONAR_HIST_SIZE;
}

int HC_SR04_GetOORCount() {
  return sonarOORCount;
}

//*****************************************************************************
//
// This is the handler for INT_GPIOB.
//
//*****************************************************************************
void GPIOB_IntHandler(void) {
  unsigned int valueGPIOb, intGPIOb;
  unsigned long currTime, diffTime;

  currTime = micros();
  valueGPIOb = GPIOPinRead(GPIO_PORTB_BASE, 0xF);
  intGPIOb = GPIOPinIntStatus(GPIO_PORTB_BASE, true);

  if (intGPIOb & GPIO_PIN_1) {
    if (valueGPIOb & GPIO_PIN_1) {
      sonarEchoStart = currTime;
      sonarEchoDur = -1;
    } else {
      diffTime = currTime - sonarEchoStart;
      if (diffTime < SONAR_DEADLINE) { // time to sound travel 5 meters (in microseconds)
        sonarEchoDur = (long)diffTime;
      }
    }
    GPIOPinIntClear(GPIO_PORTB_BASE, GPIO_PIN_1);
  }
}


void Timer2IntHandler(void) {
  TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT);

  sonarReady = true;
  sonarDistance = (float)sonarEchoDur * SONAR_SCALE;
}
