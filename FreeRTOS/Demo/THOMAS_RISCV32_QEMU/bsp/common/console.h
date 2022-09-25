/*
 * Header for the functions support console printf
 *
 * Author: Kang Zhigang <Zhigang.Kang@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
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

#endif /* __CONSOLE_H__ */
