/*
 * i2c_functions.c
 *
 *  Created on: 27/11/2014
 *      Author: Fernando1
 */

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/i2c.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include "i2c_functions.h"

unsigned long i2cError;
int i2cErrorsCount = 0;



int I2CGetErrorsCount(void) {
  return i2cErrorsCount;
}

//*****************************************************************************
//
// This function sets up I2C module as master
//
//*****************************************************************************
void I2CInit(void) {
  //
  // The I2C0 peripheral must be enabled before use.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

  //
  // For this example I2C0 is used with PortB[3:2].  The actual port and
  // pins used may be different on your part, consult the data sheet for
  // more information.  GPIO port B needs to be enabled so these pins can
  // be used.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  //
  // Configure the pin muxing for I2C0 functions on port B2 and B3.
  // This step is not necessary if your part does not support pin muxing.
  //
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);

  //
  // Select the I2C function for these pins.  This function will also
  // configure the GPIO pins pins for I2C operation, setting them to
  // open-drain operation with weak pull-ups.  Consult the data sheet
  // to see which functions are allocated per pin.
  //
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

  //
  // Enable and initialize the I2C0 master module.  Use the system clock for
  // the I2C0 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.  For this example we will use a data rate of 100kbps.
  //
  I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(), true);

}


void I2CWrite(uint8_t data, uint32_t cmd) {
  // Place the data to be sent in the data register
  I2CMasterDataPut(I2C0_MASTER_BASE, data);
  // Initiate send of data from the master.
  I2CMasterControl(I2C0_MASTER_BASE, cmd);
  // Wait until master module is done transferring.
  while (I2CMasterBusy(I2C0_MASTER_BASE)) {
  }
  // Get error status.
  i2cError = I2CMasterErr(I2C0_MASTER_BASE);
}

uint8_t I2CRead(uint32_t cmd) {
  // Tell the master to read data.
  I2CMasterControl(I2C0_MASTER_BASE, cmd);
  // Wait until the slave is done sending data.
  while (I2CMasterBusy(I2C0_MASTER_BASE)) {
  }
  // Get error status.
  i2cError = I2CMasterErr(I2C0_MASTER_BASE);
  // Read the data.
  return I2CMasterDataGet(I2C0_MASTER_BASE);
}

tBoolean I2CErrorCheck() {
  if (i2cError != I2C_MASTER_ERR_NONE) {
    // Clear interrupts
    I2CMasterIntClear(I2C0_MASTER_BASE);
    // If it was a arbitrary lost, send a stop command.
    if (i2cError == I2C_MASTER_ERR_ARB_LOST)
      I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    i2cErrorsCount++;
    return true;
  }
  return false;
}

uint8_t I2CReadMulti(uint8_t add, uint8_t *buf, size_t size) {
  int i;
  unsigned long cmd;
  // Set the address and read direction
  I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, add, true);
  // Read data.
  for (i = 0; i < size; i++) {
    if (size == 1) {
      cmd = I2C_MASTER_CMD_SINGLE_RECEIVE;
    } else if (i == 0) {
      cmd = I2C_MASTER_CMD_BURST_RECEIVE_START;
    } else if (i == size - 1) {
      cmd = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
    } else {
      cmd = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
    }
    // Read a byte.
    buf[i] = I2CRead(cmd);
    // Check for any errors
    if (I2CErrorCheck()) {
      break;
    }
  }
  return i;
}

size_t I2CReadMultiRegisters(uint8_t add, uint8_t reg, uint8_t *buf, size_t size) {
  // Set the address and write direction
  I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, add, false);
  // Register selection
  I2CWrite(reg, I2C_MASTER_CMD_BURST_SEND_START);
  // Check for any errors
  if (I2CErrorCheck())
    return 0;
  return I2CReadMulti(add, buf, size);
}

void I2CWriteRegister(uint8_t add, uint8_t reg, uint8_t val) {
  // Set the address and write direction
  I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, add, false);
  // Register selection
  I2CWrite(reg, I2C_MASTER_CMD_BURST_SEND_START);
  // Check for any errors
  if (I2CErrorCheck())
    return;
  // Value to write in register
  I2CWrite(val, I2C_MASTER_CMD_BURST_SEND_FINISH);
  // Check for any errors
  if (I2CErrorCheck())
    return;
}

uint8_t I2CReadRegister(uint8_t add, uint8_t reg) {
  uint8_t data;
  I2CReadMultiRegisters(add, reg, &data, 1);
  return data;
}

/* transform a series of bytes from big endian to little
 endian and vice versa. */
void SwapEndianness(uint8_t *buf, size_t size) {
  /* we swap in-place, so we only have to
   * place _one_ element on a temporary tray
   */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*) buf, to = &from[size - 1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}
