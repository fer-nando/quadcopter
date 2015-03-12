#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "utils/delay.h"
#include "utils/spi_functions.h"

const char *regList[] = { "CONFIG\t", "EN_AA\t", "EN_RXADDR", "SETUP_AW",
    "SETUP_RETR", "RF_CH\t", "RF_SETUP", "RX_PW_P0", "DYNPD\t", "FEATURE\t" };
const uint8_t regNum[] = { CONFIG, EN_AA, EN_RXADDR, SETUP_AW, SETUP_RETR,
    RF_CH, RF_SETUP, RX_PW_P0, DYNPD, FEATURE };
const uint8_t regCount = 10;
const char *addrList[] = { "RX_ADDR_P0", "TX_ADDR" };
const uint8_t addrNum[] = { RX_ADDR_P0, TX_ADDR };
const uint8_t addrCount = 2;

volatile uint8_t txOK;
volatile uint8_t rxOK;
volatile uint8_t rtOK;

/****************************************************************************/

void rf24WriteReg(uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t buf = W_REGISTER | reg;
  csn(0);
  SPISend(&buf, 1);
  SPISend(data, len);
  csn(PIN_CSN);
}

/****************************************************************************/

void rf24ReadReg(uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t buf = R_REGISTER | reg;
  csn(0);
  SPISend(&buf, 1);
  SPIReceive(data, len);
  csn(PIN_CSN);
}

/****************************************************************************/

void rf24WritePayload(uint8_t *payload, uint8_t len) {
  uint8_t buf = W_TX_PAYLOAD;
  csn(0);
  SPISend(&buf, 1);
  SPISend(payload, len);
  csn(PIN_CSN);
}

/****************************************************************************/

void rf24ReadPayload(uint8_t *payload, uint8_t len) {
  uint8_t buf = R_RX_PAYLOAD;
  csn(0);
  SPISend(&buf, 1);
  SPIReceive(payload, len);
  csn(PIN_CSN);
}

/****************************************************************************/

void rf24FlushTX(void) {
  uint8_t buf = FLUSH_TX;
  csn(0);
  SPISend(&buf, 1);
  csn(PIN_CSN);
}

/****************************************************************************/

void rf24FlushRX(void) {
  uint8_t buf = FLUSH_RX;
  csn(0);
  SPISend(&buf, 1);
  csn(PIN_CSN);
}

/****************************************************************************/

uint8_t rf24GetStatus(void) {
  uint8_t status;
  csn(0);
  SPIReceive(&status, 1);
  csn(PIN_CSN);
  return status;
}

/****************************************************************************/

uint8_t rf24Busy(void) {
  if (txOK) {
    return 0;
  } else if(rtOK) {
    return 1;
  } else {
    return 2;
  }
}

/****************************************************************************/

void rf24Send(uint8_t *payload, uint8_t len) {
  uint8_t buf;

  rf24ReadReg(CONFIG, &buf, 1);
  buf &= ~(1 << PRIM_RX);
  rf24WriteReg(CONFIG, &buf, 1);
  DelayUs(150);

  rf24WritePayload(payload, len);

  txOK = 0;
  rxOK = 0;
  rtOK = 1;

  ce(PIN_CE);
  //DelayUs(15);
  //ce(0);

  /*
  // Aguarda envio
  SysCtlSleep();

  if (txOK) {
    txOK = 0;
    rtOK = 1;
    return 0;
  } else if(rtOK) {
    return 1;
  } else {
    rtOK = 1;
    return 2;
  }
  */
}

/****************************************************************************/

uint8_t rf24Available(void) {
  return rxOK;
}

/****************************************************************************/

void rf24Receive(uint8_t *payload, uint8_t len) {
  rf24ReadPayload(payload, len);
  rxOK = 0;
}

/****************************************************************************/

void rf24StartListening(void) {
  uint8_t buf;

  rf24ReadReg(CONFIG, &buf, 1);
  buf |= (1 << PRIM_RX);
  rf24WriteReg(CONFIG, &buf, 1);

  rf24FlushTX();

  ce(PIN_CE);
  DelayUs(150);
}

/****************************************************************************/

void rf24StopListening(void) {
  ce(0);
  rf24FlushRX();
}

/****************************************************************************/

void rf24PowerUp(void) {
  uint8_t buf;
  rf24ReadReg(CONFIG, &buf, 1);
  buf |= (1 << PWR_UP);
  rf24WriteReg(CONFIG, &buf, 1);

  DelayUs(150);
}

/****************************************************************************/

void rf24PowerDown(void) {
  uint8_t buf;
  rf24ReadReg(CONFIG, &buf, 1);
  buf &= ~(1 << PWR_UP);
  rf24WriteReg(CONFIG, &buf, 1);
}

/****************************************************************************/

void rf24Init(void) {
  // Set CE and CSN pins as output
  GPIOPinTypeGPIOOutput(PIN_CE_BASE, PIN_CE);
  GPIOPinTypeGPIOOutput(PIN_CSN_BASE, PIN_CSN);
  ce(0);
  csn(PIN_CSN);

  // Set IRQ pin as input and enable interruption on falling
  GPIOPinTypeGPIOInput(PIN_IRQ_BASE, PIN_IRQ);
  GPIOIntTypeSet(PIN_IRQ_BASE, PIN_IRQ, GPIO_FALLING_EDGE);
  GPIOPinIntEnable(PIN_IRQ_BASE, PIN_IRQ);

  // Enable GPIOA interrupt
  IntEnable(INT_GPIOA);

  // Power on reset delay
  DelayUs(10500);

  // Flush all
  rf24FlushRX();
  rf24FlushTX();

  // Power up
  rf24PowerUp();

  // Init flags
  rxOK = 0;
  txOK = 0;
  rtOK = 1;
}

//*****************************************************************************
void rf24Setup(uint64_t addr) {
  uint8_t buf;

  // Configurações:

  // config
  rf24ReadReg(CONFIG, &buf, 1);
  buf |= (1 << CRCO);
  rf24WriteReg(CONFIG, &buf, 1);

  // endereço
  rf24WriteReg(RX_ADDR_P0, (uint8_t *) &addr, 5);
  rf24WriteReg(RX_ADDR_P1, (uint8_t *) &addr, 5);
  rf24WriteReg(TX_ADDR, (uint8_t *) &addr, 5);

  // auto retransmissão
  // delay = 1500uS, contagens = 15
  buf = (0x05 << ARD) | (0x0F << ARC);
  rf24WriteReg(SETUP_RETR, &buf, 1);

  // canal rf = 76 (2476MHz)
  buf = 76;
  rf24WriteReg(RF_CH, &buf, 1);

  // config. rf
  // data rate = 1Mbps, power = 0dBm
  // ganho LNA = ativado (economiza 0.8mA no modo RX, com perda de 1.5dB)
  buf = ~(1 << RF_DR) & ((0x03 << RF_PWR) | (1 << LNA_HCURR));
  rf24WriteReg(RF_SETUP, &buf, 1);

  // num. de bytes do payload = 8
  buf = PAYLOAD_SIZE;
  rf24WriteReg(RX_PW_P0, &buf, 1);
  rf24WriteReg(RX_PW_P1, &buf, 1);
}

/****************************************************************************/

void GPIOA_IntHandler(void) {
  uint8_t buf, status;
  uint32_t regVal;

  //UARTprintf("-> GPIO A IRQ\n");

  regVal = GPIOPinIntStatus(PIN_IRQ_BASE, true);
  if (regVal & PIN_IRQ) {
    status = rf24GetStatus();

    if ((status & (1 << RX_DR)) == (1 << RX_DR))
      rxOK = 1;
    if ((status & (1 << TX_DS)) == (1 << TX_DS))
      txOK = 1;
    if ((status & (1 << MAX_RT)) == (1 << MAX_RT))
      rtOK = 0;

    buf = status | 0x70;
    rf24WriteReg(STATUS, &buf, 1);

    rf24ReadReg(FIFO_STATUS, &buf, 1);
    if (buf & (1 << TX_EMPTY)) {
      ce(0);
    } else {
      txOK = 0;
    }

    //UARTprintf("  PA7 is low\n");

    GPIOPinIntClear(PIN_IRQ_BASE, PIN_IRQ);
  }

  //GPIOPinIntClear(GPIO_PORTA_BASE, 0xFF);
  return;
}

/****************************************************************************/

void rf24PrintAll(void) {
  uint8_t i;
  uint8_t value = 0;
  uint64_t addr = 0;

  UARTprintf("*********************\n");
  for (i = 0; i < addrCount; i++) {
    rf24ReadReg(addrNum[i], (uint8_t *) &addr, 5);
    UARTprintf("%s\t = 0x%X\n", addrList[i], addr);
  }
  UARTprintf("\n\r");
  for (i = 0; i < regCount; i++) {
    rf24ReadReg(regNum[i], &value, 1);
    UARTprintf("%s\t = 0x%X\n", regList[i], value);
  }
  UARTprintf("*********************\n\n");
}

/****************************************************************************/
