/*
 * custom.h
 *
 *  Created on: Jun 19, 2021
 *      Author: QuangVinh
 */

#ifndef INC_CUSTOM_H_
#define INC_CUSTOM_H_

#include "main.h"


void custom_clock_init();
void custom_systick_init();
void custom_delay(uint16_t miliseconds);
void custom_UART_init();
void UART_write(uint8_t data);
uint8_t UART_read();
void custom_DMA();
char find_OK();

#endif /* INC_CUSTOM_H_ */
