/*
 * Description and special function of NS16550 UART in Board-Level.
 *
 * Author: Jianheng.Zhang <zhang.jianheng@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */
#include "board-ns16550.h"
#include "interrupt.h"

struct ns16550 uart[UART_IP_NUM] = {
	{
		.ip_owner = NS_IP_OWNER_VSI,
		.base = (void *)UART0_BASE,
		.clk_freq = UART_CLK_FREQ,
		.reg_shift = 0,
		.time_out = 0,
		.en_irq_mode = false,
		.irq_num = UART0_INTR_NUM,
	},
};

struct ns16550_config ns16550_config_def = {
	.baud_rate = UART_BAUD_115200,
	.word_size = UART_LCR_WORD_LEN8,
	.parity = UART_PARITY_NONE,
	.stop_bit = UART_LCR_STOP_1BIT,
	.rx_trig_lvl = UART_RX_TRIGLVL0,
	.tx_trig_lvl = UART_TX_TRIGLVL0,
	.dma_mode = 0,
	.fifo_enable = 0,
	.intr_enable = 0,
	.flow_ctrl_enable = 0,
	.loop_back = 0,
};

const struct ns16550_reg_def ns16550_reg_def_table[UART_REG_NUM] = {
	{0x00, 0x00000000},
	{0x04, 0x00000000},
	{0x08, 0x00000001},
	{0x0c, 0x00000000},
	{0x10, 0x00000000},
	{0x14, 0x00000060},
	{0x18, 0x00000010},
	{0x1c, 0x00000000},
	{0x70, 0x00000000},
	{0x7c, 0x00000006},
	{0xa4, 0x00000000},
	{0xa8, 0x00000000},
};

void console_init(void)
{
	ns16550_uart_init(&uart[CONSOLE_UART_NUM], &ns16550_config_def);

	if (ns16550_config_def.intr_enable)
		request_irq(uart[CONSOLE_UART_NUM].irq_num,
			    uart_irq_handler, &uart[CONSOLE_UART_NUM]);
}

char console_getc(void)
{
	char c = 0;

	ns16550_uart_getc(&uart[CONSOLE_UART_NUM], &c);

	return c;
}

char console_getc_timeout(uint32_t cnt)
{
	char c = 0;

	ns16550_uart_getc_timeout(&uart[CONSOLE_UART_NUM], &c, cnt);

	return c;
}

int console_putc(char c)
{
	return ns16550_uart_putc(&uart[CONSOLE_UART_NUM], c);
}

void console_puts(const char *s)
{
	ns16550_uart_puts(&uart[CONSOLE_UART_NUM], s);
}

void uart_irq_handler(int id, void *dev)
{
	ns16550_isr_handle((struct ns16550 *)dev);
}
