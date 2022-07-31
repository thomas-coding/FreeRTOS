/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef PL011_H
#define PL011_H

typedef struct UART_t {
	volatile unsigned int UARTDR;
	volatile unsigned int UARTRSR;
			unsigned int RESERVED0[4];
	volatile unsigned int UARTFR;
			unsigned int RESERVED1;
	volatile unsigned int UARTILPR;
	volatile unsigned int UARTIBRD;
	volatile unsigned int UARTFBRD;
	volatile unsigned int UARTLCR_H;
	volatile unsigned int UARTCR;
	volatile unsigned int UARTIFLS;
	volatile unsigned int UARTIMSC;
	volatile unsigned int UARTRIS;
	volatile unsigned int UARTMIS;
	volatile unsigned int UARTICR;
	volatile unsigned int UARTDMACR;
} UART_t;

#define UART_APB_CLOCK	24000000
#define UART_BAND_RATE	115200

#define UART0_ADDR ((UART_t *)(0x40000000))

#define PL011_UARTCR_RXE          (1 << 9)	/* Receive enable */
#define PL011_UARTCR_TXE          (1 << 8)	/* Transmit enable */
#define PL011_UARTCR_LBE          (1 << 7)	/* Loopback enable */
#define PL011_UARTCR_UARTEN       (1 << 0)	/* UART Enable */

#define PL011_UARTFR_RXFE         (1 << 4)	/* Receive  FIFO empty */


#define PL011_UARTLCR_H_FEN       (1 << 4)	/* FIFOs Enable */
#define PL011_UARTLCR_H_WLEN_8    (3 << 5)
#define PL011_LINE_CONTROL  (PL011_UARTLCR_H_FEN | PL011_UARTLCR_H_WLEN_8)

void uart_init(void);
void uart_putc(char c);
char uart_getc(void);

#endif