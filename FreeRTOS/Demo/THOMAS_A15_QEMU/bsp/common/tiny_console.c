/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include "console2.h"
#include "stdarg.h"
#include <pl011.h>
#include <string.h>
#include <ctype.h>

#include "tiny_command.h"

/* #define _UC_DEBUG_ */

#define CONFIG_SYS_CBSIZE		256 /* Three standard termial line */
#define CONFIG_SYS_MAXARGS		8
#define CONFIG_SYS_PROMPT		"Alius> "
#define CONFIG_CMD_EDIT_SUPPORT

static char console_buffer[CONFIG_SYS_CBSIZE + 1];

/* Accept 0-9, a-z, A-Z, . only */
#define isvalid(c)	((c >= '0' && c <= '9') || \
			 (c >= 'a' && c <= 'z') || \
			 (c >= 'A' && c <= 'Z'))

static void backspace(void)
{
	tc_putc('\b');
	tc_putc(' ');
	tc_putc('\b');
}

#ifdef CONFIG_CMD_EDIT_SUPPORT
static char translation(char c, int esc_step)
{
	if (esc_step == 1) {
		if (c == 91)
			return 0x1b;
		else
			return 0;
	} else if (esc_step == 2) {
		switch (c) {
		case 65: /* up */
		case 66: /* down */
		case 67: /* right */
		case 68: /* left */
			return c;
		default:
			return 0;
		}
	}

	return 0;
}

static int buffer_edit(char c, char *bwp, int cur, int plen, char *tail)
{
	char *t = bwp;

	switch (c) {
	case 68: /* Left */
		if (cur == plen)
			return 0;
		t--;
		tail[strlen(tail)] = *t;
		*t = 0;
		return -1;
	case 67: /* Right */
		if (strlen(tail) == 0)
			return 0;
		*bwp = tail[strlen(tail) - 1];
		tail[strlen(tail) - 1] = '\0';
		return 1;
	default:
		return 0;
	}
}

static void show_tail(const char *tail)
{
	int i = strlen(tail);

	/* First In Last Out */
	while (--i >= 0)
		tc_putc(tail[i]);
}

#endif

/*
 * Control Key support:
 * 1. Enter
 * 2. Backspace
 * 3. Delete
 * 4. Left Key
 * 5. Right Key
 * 6. Control + C
 */
static int readline_into_buffer(const char *const prompt, char *buffer)
{
	char *bwp = buffer;	/* Buffer write pointer */
	char *bh = bwp;		/* Buffer header pointer */
	char c = 0;
	int n = 0, plen = 0;
#ifdef CONFIG_CMD_EDIT_SUPPORT
	char tail[64] = {0};
	int tlen = 0, esc_step = 0, delta = 0, cur = 0, i = 0;
#endif

	/* print prompt */
	if (prompt) {
		plen = strlen(prompt);
		tc_pr(prompt);
		n = plen;
	}

#ifdef CONFIG_CMD_EDIT_SUPPORT
	cur = n;
#endif
	for (;;) {
		c = tc_getc();
#ifdef CONFIG_CMD_EDIT_SUPPORT
		if (esc_step) {
			c = translation(c, esc_step);
			if (c == 0) {
				esc_step = 0;
				continue;
			}
		}
#endif
		switch (c) {
		case '\r':	/* Enter */
		case '\n':
#ifdef CONFIG_CMD_EDIT_SUPPORT
			while (tlen) {
				*bwp++ = tail[tlen - 1];
				tail[tlen - 1] = 0;
				tlen--;
			}
#endif
			*bwp = '\0';
			tc_pr("\n");
			return bwp - bh;
		case 0x03:	/* Control + C */
			bh[0] = '\0';
			return -1;
#ifdef CONFIG_CMD_EDIT_SUPPORT
		case 0x08:	/* Backspace */
		case 0x7f:	/* Delete */
			if (cur == plen)
				continue;
			backspace();
			*(--bwp) = 0;
			n--;
			cur--;
			if (tlen > 0) {
				show_tail(tail);
				tc_putc(' ');
				tc_putc('\b');
				i = tlen;
				while (i-- > 0)
					tc_putc('\b');
			}
			continue;
		case 0x1b:
			esc_step++;
			continue;
		case 0x41:
		case 0x42:	/* Up and Down not support */
			if (esc_step == 2) {
				esc_step = 0;
				continue;
			}
		case 0x43:	/* Right key */
		case 0x44:	/* Left key */
			if (esc_step == 2) {
				delta = buffer_edit(c, bwp, cur, plen, tail);
				bwp += delta;
				tlen -= delta;
				cur += delta;
				esc_step = 0;
				if (delta > 0)
					tc_putc(*(bwp - 1));
				else if (delta < 0)
					tc_putc('\b');
				continue;
			}
#else
		case 0x08:	/* Backspace */
		case 0x7f:
			if (n > plen) {
				*(--bwp) = 0;
				n--;
				backspace();
			}
			continue;
#endif
		default:
			if (n < CONFIG_SYS_CBSIZE - 2) {
				*bwp++ = c;
				n++;
				tc_putc(c);
#ifdef CONFIG_CMD_EDIT_SUPPORT
				cur++;
				show_tail(tail);
				i = tlen;
				while (i-- > 0)
					tc_putc('\b');
#endif
			} else {
				tc_putc('\a');
			}
		}

	}
}

static int readline(const char *const prompt)
{
	console_buffer[0] = '\0';
	return readline_into_buffer(prompt, console_buffer);
}

static int command_process(int argc, char **argv)
{
	int ret = 0;
	struct tiny_cmd *cmd = NULL;

	cmd = find_cmd(argv[0]);
	if (cmd == NULL) {
		tc_pr("## Invalid command:%s\n", argv[0]);
		return -1;
	}

	if (argc < cmd->min_args) {
		tc_pr("## Too few args\n");
		goto show_usage;
	}

	if (argc > cmd->max_args) {
		tc_pr("## Too many args\n");
		goto show_usage;
	}

	if (cmd->do_cmd)
		ret = cmd->do_cmd(cmd, argc, argv);

	if (ret != RET_CMD_USAGE)
		return ret;

show_usage:
	if (cmd->usage)
		tc_pr("%s\n", cmd->usage);

	return ret;
}

static int run_command(const char *cmd)
{
	char cmdbuf[CONFIG_SYS_CBSIZE];	/* working copy of cmd */
	char *argv[CONFIG_SYS_MAXARGS];
	int argc = 0, i = 0, len = 0, state = 0;

	if (!cmd || !*cmd)
		return -1;	/* empty command */

	if (strlen(cmd) >= CONFIG_SYS_CBSIZE) {
		tc_pr("## Command too long!\n");
		return -1;
	}

	strlcpy(cmdbuf, cmd, CONFIG_SYS_CBSIZE);

	len = strlen(cmdbuf);

#ifdef _UC_DEBUG_
	tc_pr("## CMD:%s\n", cmd);
#endif

	/* Ignore all invalid character */
	for (i = 0; i < len; i++)
		if (!isvalid(cmdbuf[i]))
			cmdbuf[i] = 0;

	i = 0;
	state = 0;
	while (i < len) {		/* State machine */
		switch (state) {
		case 0:			/* Find string header */
			if (cmdbuf[i] != 0) {
				argv[argc] = &cmdbuf[i];
				argc++;
				state = 1;
			}
			break;
		case 1:			/* Find string tail */
			if (cmdbuf[i] == 0)
				state = 0;
			break;
		}

		if (argc == CONFIG_SYS_MAXARGS) {
			tc_pr("## Too many ARGS!");
			return -1;
		}
		i++;
	}

	return command_process(argc, argv);
}

void tiny_uart_console(void)
{
	int len = 0, ret = 0;

	tc_pr("\nWelcome to alius uart console\n");

	while (1) {
		len = readline(CONFIG_SYS_PROMPT);
		if (len == -1)
			tc_pr("^C\n");
		else
			ret = run_command(console_buffer);

		if (ret == RET_CMD_EXIT)
			break;
	}
}
