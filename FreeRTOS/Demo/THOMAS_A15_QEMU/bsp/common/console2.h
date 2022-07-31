/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __CONSOLE2_H__
#define __CONSOLE2_H__
#include <stddef.h>
#include <stdint.h>

/*
 * Use the following parameter passing structure to make console_printf
 * re-entrant.
 */
struct params_s {
	uint32_t len;
	uint64_t num_integer;
	uint64_t num_decimal;
	char pad_character;
	uint8_t do_padding;
	uint8_t left_flag;
	uint8_t hex_upper;		/* Hexadecimal data output to upper case */
};

void console_printf(const char *ctrl1, ...);

#define t_printf(fmt, arg ...)		console_printf(fmt, ##arg)

#endif /* __CONSOLE2_H__ */
