/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <pl011.h>

/**
 * Initializes the UART
 */
void uart_init(void)
{

    /* 24000000/(16*115200) = 13.0208, (0.0208*16+0.5) = 2*/
    UART0_ADDR->UARTIBRD = 0xd;
    UART0_ADDR->UARTFBRD = 0x2;

	UART0_ADDR->UARTCR = PL011_UARTCR_UARTEN | PL011_UARTCR_TXE | PL011_UARTCR_RXE;
}


/**
 * Output a char to the UART TX
 */
void uart_putc(char c)
{
	UART0_ADDR->UARTDR =  c;
}


