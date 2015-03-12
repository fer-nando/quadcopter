/*
 * rc.c
 *
 *  Created on: 04/02/2015
 *      Author: Fernando
 */

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "utils/delay.h"
#include "rc.h"


volatile unsigned int rcValue[] = { 0, 0, 0, 0 };       // value of the rc input
volatile unsigned long rcEdgeTime[] = { 0, 0, 0, 0 };   // rising time


//*****************************************************************************
//
// Initialize and configure peripherals used to capture RC input.
//
//*****************************************************************************
void RCInit() {

  // Enable the peripherals used.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

  // Configure the PC4-PC7 to be inputs to indicate entry of one
  // of the interrupt handlers.
  GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,
     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
  GPIOIntTypeSet(GPIO_PORTC_BASE,
     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
  GPIOPinIntEnable(GPIO_PORTC_BASE,
     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

  // Enable the interrupts.
  IntEnable(INT_GPIOC);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void RCGetValues(int * rc) {
  rc[0] = rcValue[0];
  rc[1] = rcValue[1];
  rc[2] = rcValue[2];
  rc[3] = rcValue[3];
}


//*****************************************************************************
//
// This is the handler for INT_GPIOC.
//
//*****************************************************************************
#define RCCheck(num,pin)                          \
    if (intGPIOc & pin) {                         \
      if (valueGPIOc & pin) {                     \
        rcEdgeTime[num] = currTime;               \
      } else {                                    \
        diffTime = currTime - rcEdgeTime[num];    \
        if (800 < diffTime && diffTime < 2200) {  \
          rcValue[num] = diffTime;                \
        }                                         \
      }                                           \
      GPIOPinIntClear(GPIO_PORTC_BASE, pin);      \
    }

void GPIOC_IntHandler(void) {
  unsigned int valueGPIOc, intGPIOc;
  unsigned long currTime, diffTime;

  currTime = micros();
  valueGPIOc = GPIOPinRead(GPIO_PORTC_BASE, 0xFF);
  intGPIOc = GPIOPinIntStatus(GPIO_PORTC_BASE, true);

  //RCCheck(0, GPIO_PIN_4);
  if (intGPIOc & GPIO_PIN_4) {
    if (valueGPIOc & GPIO_PIN_4) {
      rcEdgeTime[0] = currTime;
    } else {
      diffTime = currTime - rcEdgeTime[0];
      if (800 < diffTime && diffTime < 2200) {
        rcValue[0] = diffTime;
      }
    }
    GPIOPinIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
  }
  RCCheck(1, GPIO_PIN_5);
  RCCheck(2, GPIO_PIN_7);
  RCCheck(3, GPIO_PIN_6);
}
