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


volatile int      rcValue[4] = { 0, 0, 0, 0 };      // value of the rc input
volatile uint64_t rcEdgeTime[4] = { 0, 0, 0, 0 };   // rising time
volatile tBoolean rcUpdated[4] = { false, false, false, false };

int rcff[4][3], rcfb[4][3];


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

void RCLowPassFilter(int num) {
  int i;
  int * x = rcff[num];
  int * y = rcfb[num];
  // swift previous data
  for (i = 2; i > 0; i--) {
    x[i] = x[i-1];
    y[i] = y[i-1];
  }
  // apply 5Hz low pass filter
  x[0] = rcValue[num];
  y[0] = 0.1729*x[0] + 0.3458*x[1] + 0.1729*x[2]
                     + 0.5301*y[1] - 0.2217*y[2];
  rcValue[num] = y[0];
}

//*****************************************************************************
//
//
//
//*****************************************************************************
tBoolean RCGetValues(int * rc) {
  tBoolean updated = false;
  int i;
  for (i = 0; i < 4; i++) {
    if (rcUpdated[i]) {
      rcUpdated[i] = false;
      RCLowPassFilter(i);
      updated = true;
    }
    rc[i] = rcValue[i];
  }
  return updated;
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
          rcUpdated[num] = true;                  \
        }                                         \
      }                                           \
      GPIOPinIntClear(GPIO_PORTC_BASE, pin);      \
    }

void GPIOC_IntHandler(void) {
  unsigned long valueGPIOc, intGPIOc;
  uint64_t currTime, diffTime;

  currTime = micros();
  valueGPIOc = GPIOPinRead(GPIO_PORTC_BASE, 0xFF);
  intGPIOc = GPIOPinIntStatus(GPIO_PORTC_BASE, true);

  RCCheck(0, GPIO_PIN_4);
  RCCheck(1, GPIO_PIN_5);
  RCCheck(2, GPIO_PIN_7);
  RCCheck(3, GPIO_PIN_6);
}
