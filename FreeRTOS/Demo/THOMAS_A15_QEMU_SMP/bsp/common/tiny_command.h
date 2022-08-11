/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TINY_COMMAND_H
#define TINY_COMMAND_H

#include "pl011.h"

#define ARRAY_SIZE(x)			(sizeof(x) / sizeof((x)[0]))

#define tc_tolower(c)	((c) >= 'A' && (c) <= 'Z') ? ((c) - 'A' + 'a') : (c)
#define tc_toupper(c)	((c) >= 'a' && (c) <= 'z') ? ((c) - 'a' + 'A') : (c)
#define tc_isdigit(c)	(((c) >= '0' && (c) <= '9'))
#define tc_isxdigit(c)	(((c) >= '0' && (c) <= '9') || \
			((c) >= 'a' && (c) <= 'f') || \
			((c) >= 'A' && (c) <= 'F'))
#define tc_islower(c)	((c) >= 'a' && (c) <= 'f')

#define tc_putc(c)			uart_putc(c)
#define tc_getc(c)			uart_getc(c)
#define tc_pr(format, args...)		console_printf(format, ##args)

typedef void (*func_entry)(void);

#define RET_CMD_USAGE	-1
#define RET_CMD_DONE	0
#define RET_CMD_EXIT	1

struct tiny_cmd {
	char *name;
	unsigned int min_args;
	unsigned int max_args;
	int (*do_cmd)(struct tiny_cmd *cmd, int argc, char *argv[]);
	char *usage;
};

#define TINY_CMD(_name, _min_args, _max_args, _handler, _usage) \
	{	\
		.name = (_name), \
		.min_args = (_min_args), \
		.max_args = (_max_args), \
		.do_cmd = (_handler), \
		.usage = (_usage), \
	}

struct tiny_cmd *find_cmd(const char *name);

#endif
