/*
 * Generic driver code for NS16550 UART
 *
 * Author: Jianheng.Zhang <zhang.jianheng@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */
#include "common.h"
#include "ns16550.h"

static void ns16550_writeb(struct ns16550 *ns_uart, int32_t offset, int32_t value)
{
	uint8_t *addr = NULL;

	offset *= 1 << ns_uart->reg_shift;
	addr = (uint8_t *)ns_uart->base + offset;
	write_mreg8(addr, value);
}

static int32_t ns16550_readb(struct ns16550 *ns_uart, int32_t offset)
{
	uint8_t *addr = NULL;

	offset *= 1 << ns_uart->reg_shift;
	addr = (uint8_t *)ns_uart->base + offset;

	return read_mreg8(addr);
}

void ns16550_isr_handle(struct ns16550 *ns_uart)
{
	uint32_t uart_intr_flag = 0;

	uart_intr_flag = 0xf & ns16550_readb(ns_uart, UART_IIR);

	if (uart_intr_flag == UART_INTR_ID_TI) {
		/* THR empty */
		ns_uart->priv_data.intr_tflag = UART_INTR_ID_TI;
	} else if (uart_intr_flag == UART_INTR_ID_RDI) {
		/* RHR available */
		ns_uart->priv_data.irq_rx = ns16550_readb(ns_uart, UART_RBR);
		ns_uart->priv_data.intr_rflag = UART_INTR_ID_RDI;
	} else if (uart_intr_flag == UART_INTR_ID_TIMEOUT) {
		/* Character timeout indication */
	} else if (uart_intr_flag == UART_INTR_ID_RLSI) {
		/*
		 * Overrun/parity/framing errors, break interrupt, or address
		 * received interrupt, reading the line status register to know
		 * what happened.
		 */
		ns16550_readb(ns_uart, UART_LSR);
	}
}

static void ns16550_set_baudrate(struct ns16550 *ns_uart, uint32_t baudrate)
{
	uint32_t div = 0;
	float apb_bus_freq = ns_uart->clk_freq;
	float baud = baudrate;

	if (baud == 0)
		baud = UART_BAUD_115200;
	div = (uint32_t)(apb_bus_freq / (baud * 16) + 0.5);

	/* switch to access DLL&DLM */
	ns16550_writeb(ns_uart, UART_LCR, UART_LCR_DLAB);
	ns16550_writeb(ns_uart, UART_DLL, ((div >> 0) & 0xff));
	ns16550_writeb(ns_uart, UART_DLM, ((div >> 8) & 0xff));
}

int32_t ns16550_uart_init(struct ns16550 *ns_uart, struct ns16550_config *config)
{
	uint8_t tmp8 = 0;

	/* SNPS UART Software Reset */
	if (ns_uart->ip_owner & NS_IP_OWNER_SNPS)
		ns16550_writeb(ns_uart, UART_SRR, 0x7);

	ns16550_set_baudrate(ns_uart, config->baud_rate);
	ns_uart->priv_data.parity_en = config->parity ? 1 : 0;
	/* switch to access THR&IER, and uart config */
	tmp8 = (config->word_size) | (config->parity) | (config->stop_bit);
	ns16550_writeb(ns_uart, UART_LCR, tmp8);
	tmp8 = (config->rx_trig_lvl << 6) | (config->tx_trig_lvl << 4)
		| (config->dma_mode << 3) | (config->fifo_enable);

	/* If FIFOs are not implemented, Auto Flow Control cannot be selected */
	if (config->flow_ctrl_enable && (tmp8 & config->fifo_enable) == 0)
		tmp8 |= config->fifo_enable;

	/* trig level set, dma mode and enable fifo and reset fifo */
	ns16550_writeb(ns_uart, UART_FCR, tmp8 | 0x06);

	/* flow control config */
	tmp8 = (config->flow_ctrl_enable << 5) | (config->loop_back << 4);
	if (config->flow_ctrl_enable)
		tmp8 |= (0x1 << 1);	/* Request to Send */
	ns16550_writeb(ns_uart, UART_MCR, tmp8);

	if (config->intr_enable) {
		ns_uart->priv_data.intr_tflag = UART_INTR_ID_TI;
		ns_uart->priv_data.intr_rflag = 0;
		ns_uart->priv_data.irq_rx = 0;
		ns16550_writeb(ns_uart, UART_IER, UART_IER_ERDI | UART_IER_ETI);
		ns_uart->en_irq_mode = true;
	} else {
		ns16550_writeb(ns_uart, UART_IER, 0x0);
		ns_uart->en_irq_mode = false;
	}

	return 0;
}

static int32_t ns16550_uart_putc_intr(struct ns16550 *ns_uart, char c)
{
	uint32_t count = ns_uart->time_out;
	int32_t ret = -1;

	while ((ns_uart->priv_data.intr_tflag != UART_INTR_ID_TI)
		&& (ns_uart->time_out ? (count-- != 0) : 1))
		udelay(10);

	ns_uart->priv_data.intr_tflag = 0;

	if ((count != 0) || (ns_uart->time_out == 0)) {
		ns16550_writeb(ns_uart, UART_THR, (uint8_t)c);
		ret = 0;
	} else {
		return ret;
	}

	while ((ns_uart->priv_data.intr_tflag != UART_INTR_ID_TI)
		&& (ns_uart->time_out ? (count-- != 0) : 1))
		udelay(10);

	return ret;
}

static int32_t ns16550_uart_putc_polling(struct ns16550 *ns_uart, char c)
{
	uint32_t count = ns_uart->time_out;
	int32_t val = -1;
	int32_t ret = -1;

	do {
		val = ns16550_readb(ns_uart, UART_LSR);
	} while (((val & 0x20) == 0x00) /* transmit holding register empty */
		&& ((count != 0) || (ns_uart->time_out == 0)));

	if ((count != 0) || (ns_uart->time_out == 0)) {
		ns16550_writeb(ns_uart, UART_THR, (uint8_t)c);
		ret = 0;
	} else {
		return ret;
	}

	do {
		val = ns16550_readb(ns_uart, UART_LSR);
	} while (((val & 0x20) == 0x00) /* transmit holding register empty */
		&& ((count != 0) || (ns_uart->time_out == 0)));

	return ret;
}

static int32_t ns16550_uart_getc_intr(struct ns16550 *ns_uart, char *c)
{
	uint32_t count = ns_uart->time_out;
	uint8_t val = 0;

	while (((ns_uart->priv_data.intr_rflag != UART_INTR_ID_RDI)
		&& (ns_uart->priv_data.intr_rflag != UART_INTR_ID_TIMEOUT))
		&& ((count != 0) || (ns_uart->time_out == 0)))
		;

	if ((ns_uart->priv_data.intr_rflag == UART_INTR_ID_RDI)
		&& ((count != 0) || (ns_uart->time_out == 0))) {
		if (ns_uart->priv_data.parity_en
			&& (ns16550_readb(ns_uart, UART_LSR) & 0x04)) {
			ns_uart->priv_data.intr_rflag = 0;
			return -1;
		}

		*c = ns_uart->priv_data.irq_rx;
	} else {
		do {
			val = ns16550_readb(ns_uart, UART_LSR);
			ns16550_readb(ns_uart, UART_RBR); /* read empty data */
		} while ((val & 0x01) != 0x00);
	}
	ns_uart->priv_data.intr_rflag = 0;

	return 0;
}

static int32_t ns16550_uart_getc_polling(struct ns16550 *ns_uart, char *c)
{
	uint32_t count = ns_uart->time_out;
	uint32_t val = 0;

	do {
		val = ns16550_readb(ns_uart, UART_LSR);
	} while (((val & 0x1) == 0x00) /* transmit holding register empty */
		&& ((count != 0) || (ns_uart->time_out == 0)));

	if (ns_uart->priv_data.parity_en && (val & 0x04))
		return -1;

	if ((count != 0) || (ns_uart->time_out == 0)) {
		*c = ns16550_readb(ns_uart, UART_RBR);
		return 0;
	}

	return -1;
}

int32_t ns16550_uart_getc_timeout(struct ns16550 *ns_uart, char *c, uint32_t timeout)
{
	uint32_t count = timeout;
	uint32_t val = 0;

	do {
		val = ns16550_readb(ns_uart, UART_LSR);
	} while (((val & 0x1) == 0x00) && ((count-- != 0)));

	if (ns_uart->priv_data.parity_en && (val & 0x04))
		return -1;

	if ((count != 0) || (ns_uart->time_out == 0)) {
		*c = ns16550_readb(ns_uart, UART_RBR);
		return 0;
	}

	return -1;
}

int32_t ns16550_uart_getc(struct ns16550 *ns_uart, char *c)
{
	if (c == NULL)
		return -1;

	if (ns_uart->en_irq_mode)
		return ns16550_uart_getc_intr(ns_uart, c);
	else
		return ns16550_uart_getc_polling(ns_uart, c);
}

int32_t ns16550_uart_putc(struct ns16550 *ns_uart, char c)
{
	if (ns_uart->en_irq_mode)
		return ns16550_uart_putc_intr(ns_uart, c);
	else
		return ns16550_uart_putc_polling(ns_uart, c);
}

void ns16550_uart_puts(struct ns16550 *ns_uart, const char *s)
{
	if (s == NULL)
		return;

	while (*s) {
		if (*s == '\n') {
			ns16550_uart_putc(ns_uart, '\r');
			ns16550_uart_putc(ns_uart, '\n');
		} else {
			ns16550_uart_putc(ns_uart, *s);
		}
		s++;
	}
}

int32_t ns16550_register_default(struct ns16550 *ns_uart,
			const struct ns16550_reg_def *table,
			struct ns16550_reg_def *comp, uint32_t num)
{
	uint32_t i = 0;
	uint32_t val = 0;
	int32_t ret = 0;

	for (i = 0; i < num; i++) {
		val = read_mreg32(ns_uart->base + table[i].off);
		if (val != table[i].def) {
			ret = -1;
			comp[i].off = 0xffffffff;
			comp[i].def = val;
		}
	}

	return ret;
}
