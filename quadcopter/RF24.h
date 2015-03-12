
#ifndef __RF24_H
#define __RF24_H


#include <stdint.h>
#include "nRF24L01.h"


#define RF_SENT         0
#define RF_SENDING      1
#define RF_SEND_FAIL    2
#define RF_LISTENING    3
#define RF_FIFO_FULL    (1<<FIFO_FULL)

#define PAYLOAD_SIZE    27

#define PIN_CE_BASE 	  GPIO_PORTA_BASE
#define PIN_CSN_BASE    GPIO_PORTA_BASE
#define PIN_IRQ_BASE 	  GPIO_PORTA_BASE

#define PIN_CE  		    GPIO_PIN_6
#define PIN_CSN         GPIO_PIN_3
#define PIN_IRQ 		    GPIO_PIN_7

#define ce(x) 			    (GPIOPinWrite(PIN_CE_BASE, PIN_CE, x))
#define csn(x) 			    (GPIOPinWrite(PIN_CSN_BASE, PIN_CSN, x))

void rf24Init();
void rf24Setup(uint64_t addr);
void rf24WriteReg(uint8_t reg, uint8_t *data, uint8_t len);
void rf24ReadReg(uint8_t reg, uint8_t *data, uint8_t len);
void rf24WritePayload(uint8_t *payload, uint8_t len);
void rf24ReadPayload(uint8_t *payload, uint8_t len);
void rf24FlushTX(void);
void rf24FlushRX(void);
uint8_t rf24GetStatus(void);
uint8_t rf24GetFIFOStatus(void);
uint8_t rf24Busy(void);
uint8_t rf24Available(void);
uint64_t rf24GetTxCounter(void);
void rf24Send(uint8_t *payload, uint8_t len);
void rf24Receive(uint8_t *payload, uint8_t len);
void rf24StartListening(void);
void rf24StopListening(void);
void rf24PowerUp(void);
void rf24PowerDown(void);
void rf24PrintAll(void);


#endif
