/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CONSOLE_H
#define CONSOLE_H

void console_init(void);
void bm_printf(char *s);
void bm_printf_value_u32(char *s, unsigned int value);
void bm_printf_value_u8(char *s, unsigned char value);

#endif