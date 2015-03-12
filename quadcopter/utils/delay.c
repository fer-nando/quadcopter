#include <limits.h>
#include <inc/hw_types.h>
#include "inc/hw_memmap.h"
#include "driverlib/timer.h"
#include <driverlib/sysctl.h>
#include "delay.h"

uint64_t micros() {
  return (UINT32_MAX - TimerValueGet(WTIMER0_BASE, TIMER_A));
}

void DelayUs(uint64_t value) {
  SysCtlDelay(SysCtlClockGet() / 3 / 1000000 * value);
}

void DelayMs(uint64_t value) {
  DelayUs(value * 1000);
}

void DelayInit() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

  TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

  TimerLoadSet(WTIMER0_BASE, TIMER_A, 0xFFFFFFFF);

  TimerPrescaleSet(WTIMER0_BASE, TIMER_A, SysCtlClockGet()/1000000 - 1);

  TimerEnable(WTIMER0_BASE, TIMER_A);
}
