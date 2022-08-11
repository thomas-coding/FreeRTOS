/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <pl011.h>
#include "stdarg.h"
#include <string.h>
#include <ctype.h>
#include <stdint.h>

/**
 * Initializes the UART
 */
void uart_init(void)
{

    /* 24000000/(16*115200) = 13.0208, (0.0208*16+0.5) = 2*/
    UART0_ADDR->UARTIBRD = 0xd;
    UART0_ADDR->UARTFBRD = 0x2;

    UART0_ADDR->UARTLCR_H = PL011_LINE_CONTROL;
    UART0_ADDR->UARTCR = PL011_UARTCR_UARTEN | PL011_UARTCR_TXE | PL011_UARTCR_RXE;
}

/**
 * Output a char to the UART TX
 */
void uart_putc(char c)
{
    UART0_ADDR->UARTDR =  c;
}

/**
 * get a char
 */
char uart_getc(void)
{
    uint32_t val = 0;

    do {
        val = UART0_ADDR->UARTFR;
    } while ((val & PL011_UARTFR_RXFE) != 0x00); /* wait for rx fifo not empty */

    return UART0_ADDR->UARTDR;
}

