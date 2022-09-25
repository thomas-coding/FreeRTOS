/*
 * Header for description and special function of NS16550 UART in Board-Level.
 *
 * Author: Jianheng.Zhang <zhang.jianheng@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#ifndef __BOARD_UART_H__
#define __BOARD_UART_H__

#include "ns16550.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <addr_map.h>

/* indicating how many UART in this SoC */
#define UART_IP_NUM		1

#define CONSOLE_UART_NUM	0

/* indicating the frequency of UART */
#define UART_CLK_FREQ		24000000

/* indicating how many registers for each UART */
#define UART_REG_NUM		12

/*
 * indicating the interrupt number of each UART
 */
#define UART0_INTR_NUM		20

extern struct ns16550 uart[UART_IP_NUM];
extern struct ns16550_config ns16550_config_def;
extern const struct ns16550_reg_def ns16550_reg_def_table[UART_REG_NUM];

char console_getc(void);
char console_getc_timeout(uint32_t cnt);
int console_putc(char c);
void console_puts(const char *s);
void console_init(void);
void uart_irq_handler(int id, void *dev);

#endif /* __BOARD_UART_H__ */
