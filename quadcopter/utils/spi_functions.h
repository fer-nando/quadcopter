/*
 * spi_functions.h
 *
 *  Created on: 10/02/2015
 *      Author: Fernando
 */

#ifndef SPI_FUNCTIONS_H_
#define SPI_FUNCTIONS_H_


#include <stdint.h>


void SPIInit(void);
void SPISend(uint8_t *data, uint8_t len);
void SPIReceive(uint8_t *data, uint8_t len);


#endif /* SPI_FUNCTIONS_H_ */
