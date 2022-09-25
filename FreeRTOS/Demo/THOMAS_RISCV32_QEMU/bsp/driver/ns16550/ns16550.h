/*
 * Generic driver code for NS16550 UART
 *
 * Author: Jianheng.Zhang <zhang.jianheng@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#ifndef __NS16550_H__
#define __NS16550_H__
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* IP owner */
#define NS_IP_OWNER_VSI		0x00	/* VeriSilicion */
#define NS_IP_OWNER_SNPS	0x01	/* Synopsys */

/* NS16550 specification */
#define UART_RBR		0x000	/* Receiver Buffer Register */
#define UART_THR		0x000	/* Transmitter Holding Register */
#define UART_IER		0x004	/* Interrupt Enable Register */
#define UART_IIR		0x008	/* Interrupt Identification Reg */
#define UART_FCR		0x008	/* FIFO Control Register */
#define UART_LCR		0x00c	/* Line Control Register */
#define UART_MCR		0x010	/* Modem Control Register */
#define UART_LSR		0x014	/* Line Status Register */
#define UART_MSR		0x018	/* Modem Status Register */
#define UART_SCR		0x01c	/* Scratch Register */
#define UART_DLL		0x000	/* Divisor Latch LSB */
#define UART_DLM		0x004	/* Divisor Latch MSB */

/* SNPS: Software Reset Register */
#define UART_SRR		0x088

/* UART_LCR bit define */
#define UART_LCR_WORD_LEN5	(0 << 0)
#define UART_LCR_WORD_LEN6	(1 << 0)
#define UART_LCR_WORD_LEN7	(2 << 0)
#define UART_LCR_WORD_LEN8	(3 << 0)
#define UART_LCR_STOP_1BIT	(0 << 2)
#define UART_LCR_STOP_2BIT	(1 << 2)
#define UART_LCR_PE		(1 << 3)
#define UART_LCR_EPS		(1 << 4)
#define UART_LCR_SP		(1 << 5)
#define UART_LCR_BC		(1 << 6)
#define UART_LCR_DLAB		(1 << 7)

/* UART_IER bit define */
#define UART_IER_ERDI		(1 << 0)
#define UART_IER_ETI		(1 << 1)
#define UART_IER_ERLSI		(1 << 2)
#define UART_IER_EMSI		(1 << 3)

/* UART interrupt ID */
#define UART_INTR_ID_RLSI	0x06
#define UART_INTR_ID_RDI	0x04
#define UART_INTR_ID_TIMEOUT	0x0c
#define UART_INTR_ID_TI		0x02

/* UART trigger level */
#define UART_RX_TRIGLVL0	0x00	/* 1 char in the fifo */
#define UART_RX_TRIGLVL1	0x01	/* fifo 1/4 full */
#define UART_RX_TRIGLVL2	0x02	/* fifo 1/2 full */
#define UART_RX_TRIGLVL3	0x03	/* fifo 1/8 less full */
#define UART_TX_TRIGLVL0	0x00	/* fifo empty */
#define UART_TX_TRIGLVL1	0x01	/* 2 chars in the fifo */
#define UART_TX_TRIGLVL2	0x02	/* fifo 1/4 full */
#define UART_TX_TRIGLVL3	0x03	/* fifo 1/2 full */

/* UART baud rate setting */
#define UART_BAUD_115200	115200
#define UART_BAUD_57600		57600
#define UART_BAUD_38400		38400
#define UART_BAUD_19200		19200
#define UART_BAUD_9600		9600
#define UART_BAUD_2M		2000000
#define UART_BAUD_1M		1000000
#define UART_BAUD_DIV1		1500000

/* UART Parity */
#define UART_PARITY_NONE	(0 << 3)
#define UART_PARITY_ODD		(1 << 3)
#define UART_PARITY_EVEN	(3 << 3)

struct ns16550_config {
	uint32_t baud_rate;
	uint8_t word_size;
	uint8_t parity;
	uint8_t stop_bit;
	uint8_t rx_trig_lvl;
	uint8_t tx_trig_lvl;
	uint8_t dma_mode;
	uint8_t intr_enable;
	uint8_t fifo_enable;
	uint8_t flow_ctrl_enable;
	uint8_t loop_back;
};

struct ns16550_priv {
	uint32_t intr_tflag;
	uint32_t intr_rflag;
	char irq_rx;
	uint8_t parity_en;
};

struct ns16550 {
	uint32_t ip_owner;
	void *base;
	uint32_t clk_freq;
	uint8_t reg_shift;
	uint32_t time_out;
	uint8_t en_irq_mode;
	int irq_num;
	struct ns16550_priv priv_data;
};

struct ns16550_reg_def {
	uint32_t off;
	uint32_t def;
};

int32_t ns16550_uart_getc(struct ns16550 *ns_uart, char *c);
int32_t ns16550_uart_getc_timeout(struct ns16550 *ns_uart, char *c, uint32_t timeout);
int32_t ns16550_uart_putc(struct ns16550 *ns_uart, char c);
void ns16550_uart_puts(struct ns16550 *ns_uart, const char *s);
int32_t ns16550_uart_init(struct ns16550 *ns_uart,
			struct ns16550_config *config);
void ns16550_isr_handle(struct ns16550 *ns_uart);
int32_t ns16550_register_default(struct ns16550 *ns_uart,
			const struct ns16550_reg_def *table,
			struct ns16550_reg_def *comp, uint32_t num);

#endif /* #ifndef __NS16550_H__ */
