/*
 * spi_functions.c
 *
 *  Created on: 10/02/2015
 *      Author: Fernando
 */

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "spi_functions.h"


//*****************************************************************************
//
// This function sets up SSI module as SPI
//
//*****************************************************************************
void SPIInit(void) {
  //
  // The SSI0 peripheral must be enabled for use.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

  //
  // For this example SSI0 is used with PortA[5:2].  The actual port and pins
  // used may be different on your part, consult the data sheet for more
  // information.  GPIO port A needs to be enabled so these pins can be used.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //
  // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
  // This step is not necessary if your part does not support pin muxing.
  //
  GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  //GPIOPinConfigure(GPIO_PA3_SSI0FSS); // FSS pin will be set manually
  GPIOPinConfigure(GPIO_PA4_SSI0RX);
  GPIOPinConfigure(GPIO_PA5_SSI0TX);

  //
  // Configure the GPIO settings for the SSI pins.  This function also gives
  // control of these pins to the SSI hardware.  Consult the data sheet to
  // see which functions are allocated per pin.
  // The pins are assigned as follows:
  //      PA5 - SSI0Tx
  //      PA4 - SSI0Rx
  //      PA3 - SSI0Fss
  //      PA2 - SSI0CLK
  //
  GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);

  //
  // Configure and enable the SSI port for SPI master mode.  Use SSI0,
  // system clock supply, idle clock level low and active low clock in
  // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
  // For SPI mode, you can set the polarity of the SSI clock when the SSI
  // unit is idle.  You can also configure what clock edge you want to
  // capture data on.  Please reference the datasheet for more information on
  // the different SPI modes.
  //
  SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
      SSI_MODE_MASTER, 1000000, 8);

  //
  // Enable the SSI0 module.
  //
  SSIEnable(SSI0_BASE);
}

/****************************************************************************/
void SPISend(uint8_t *data, uint8_t len) {
  uint8_t i;
  unsigned long ulData, dummyData;
  for (i = 0; i < len; i++) {
    ulData = (unsigned long) data[i];
    // put data on transmit FIFO - blocking
    SSIDataPut(SSI0_BASE, ulData);
    // read dummy data
    SSIDataGet(SSI0_BASE, &dummyData);
  }
  // wait until data is transferred
  while (SSIBusy(SSI0_BASE)) {
  }
}

/****************************************************************************/
void SPIReceive(uint8_t *data, uint8_t len) {
  uint8_t i;
  unsigned long ulData;
  unsigned long dummyData = 0xFF;
  for (i = 0; i < len; i++) {
    // write dummy data
    SSIDataPut(SSI0_BASE, dummyData);
    // get data from receive FIFO - blocking
    SSIDataGet(SSI0_BASE, &ulData);
    data[i] = (uint8_t) ulData;
  }
}
