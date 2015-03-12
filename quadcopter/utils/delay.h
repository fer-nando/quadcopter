#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

uint64_t micros();
void DelayUs(uint64_t value);
void DelayMs(uint64_t value);
void DelayInit();

#endif
