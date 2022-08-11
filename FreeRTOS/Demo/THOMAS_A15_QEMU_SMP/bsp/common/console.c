/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <pl011.h>
#include <interrupt.h>

#define U32_MAX_DIGIT_BITS 12  /* 8 char for number, 2 for '0x' and 1 for '\n', last 0 for end */
#define U8_MAX_DIGIT_BITS 6  /* 2 char for number, 2 for '0x' and 1 for '\n', last 0 for end */
static const char HEX_TABLE[] = {'0', '1', '2', '3', '4', '5', '6', '7',
								'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
int print_core_id = 1;

void bm_printf(char *s)
{
	int cpu_id = 0;
	cpu_id = cpu_id_get();

	if(print_core_id == 1) {
		uart_putc('[');
		uart_putc(HEX_TABLE[cpu_id]);
		uart_putc(']');
	}
	while (*s) {
		uart_putc(*s);
		s++;
	}
}

static void to_hex_u32(unsigned int value, char msg[])
{
	int i = U32_MAX_DIGIT_BITS - 1;

	msg[i--] = 0;
	msg[i--] = '\n';
	for (; i > 1; i--, value >>= 4)
		msg[i] = HEX_TABLE[value & 0xF];
	msg[i--] = 'x';
	msg[i--] = '0';
}

static void to_hex_u8(unsigned char value, char msg[])
{
	int i = U8_MAX_DIGIT_BITS - 1;

	msg[i--] = 0;
	msg[i--] = '\n';
	for (; i > 1; i--, value >>= 4)
		msg[i] = HEX_TABLE[value & 0xF];
	msg[i--] = 'x';
	msg[i--] = '0';
}

static void bm_printf_hex_u32(unsigned int value)
{
	char value_str[U32_MAX_DIGIT_BITS];

	for (int i = 0; i < U32_MAX_DIGIT_BITS; i++)
		value_str[i] = 0;

	to_hex_u32(value, value_str);
	bm_printf(value_str);
}

static void bm_printf_hex_u8(unsigned char value)
{
	char value_str[U8_MAX_DIGIT_BITS];

	for (int i = 0; i < U8_MAX_DIGIT_BITS; i++)
		value_str[i] = 0;

	to_hex_u8(value, value_str);
	bm_printf(value_str);
}

void bm_printf_value_u32(char *s, unsigned int value)
{
	//interrupts_disable();
	bm_printf(s);
	print_core_id = 0;
	bm_printf_hex_u32(value);
	print_core_id = 1;
	//interrupts_enable();
}

void bm_printf_value_u8(char *s, unsigned char value)
{
	//interrupts_disable();
	bm_printf(s);
	print_core_id = 0;
	bm_printf_hex_u8(value);
	print_core_id = 1;
	//interrupts_enable();
}

void console_init(void)
{
	uart_init();
}
