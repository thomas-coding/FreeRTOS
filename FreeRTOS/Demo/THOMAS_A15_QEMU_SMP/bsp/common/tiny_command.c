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

#define DEFAULT_BAUDRATE	UART_BAUD_115200

static unsigned long tc_strtoul(const char *cp, char **endp,
					unsigned int base);

static void *map_sysmem(unsigned long vaddr, unsigned long len);
static void unmap_sysmem(void *paddr);

/* Command handler */
static int do_mw(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_mr(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_md(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_cmp(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_cp(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_go(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_exit(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_help(struct tiny_cmd *cmd, int argc, char *argv[]);
static int do_run_blink(struct tiny_cmd *cmd, int argc, char *argv[]);

/* Command table */
struct tiny_cmd tc_cmd_tb[] = {
	TINY_CMD("mw", 3, 4, do_mw,
		"mw addr value [width]         - Memory Write One Unit in Width"),
	TINY_CMD("mr", 2, 3, do_mr,
		"mr addr [width]               - Memory Read One Unit in Width"),
	TINY_CMD("md", 3, 3, do_md,
		"md addr len                   - Memory Dump in words"),
	TINY_CMD("cp", 4, 4, do_cp,
		"cp dst src len                - Memory copy in bytes"),
	TINY_CMD("cmp", 4, 4, do_cmp,
		"cmp addr0 addr1 len           - Memory Compare"),
	TINY_CMD("go", 2, 2, do_go,
		"go addr                       - Jump to run"),
	TINY_CMD("blink", 1, 1, do_run_blink,
		"blink                       - Jump to run blinkdemo"),
	TINY_CMD("exit", 1, 1, do_exit,
		"exit                          - Exit console"),
	TINY_CMD("help", 1, 2, do_help,
		"help                          - Help information"),
	/* Add new command implementation here */
};

#define TC_CMD_TB_LEN	ARRAY_SIZE(tc_cmd_tb)

int do_cp(struct tiny_cmd *cmd, int argc, char *argv[])
{
	uint32_t src = 0;
	uint32_t dst = 0;
	unsigned long len = 0;

	dst = tc_strtoul(argv[1], NULL, 16);
	src = tc_strtoul(argv[2], NULL, 16);
	len = tc_strtoul(argv[3], NULL, 16);

	memcpy((void *)dst, (void *)src, len);

	return RET_CMD_DONE;
}

/*
 * Exit console loop and continue to execute the following flow.
 */
int do_exit(struct tiny_cmd *cmd, int argc, char *argv[])
{
	/* Notify the console loop to break. */
	tc_pr("Goodbye tiny console\n");

	return RET_CMD_EXIT;
}

/*
 * Help
 */
static int do_help(struct tiny_cmd *cmd, int argc, char *argv[])
{
	int i = 0;

	for (i = 0; i < TC_CMD_TB_LEN; i++)
		tc_pr("%s\n", tc_cmd_tb[i].usage);

	return RET_CMD_DONE;
}

/*
 * Memory Write
 * - mw addr value [width]
 */
static int do_mw(struct tiny_cmd *cmd, int argc, char *argv[])
{
	unsigned int width = 32;
	unsigned long writeval = 0;
	unsigned long addr = 0;
	void *buf = 0;

	addr = tc_strtoul(argv[1], NULL, 16);

	if (argc == 4)
		width = tc_strtoul(argv[3], NULL, 10);

	if (width != 8 && width != 16 && width != 32)
		return RET_CMD_USAGE;

	writeval = tc_strtoul(argv[2], NULL, 16);
	buf = map_sysmem(addr, 4);

	if (width == 8)
		*(volatile uint8_t*)buf = (uint8_t)writeval;
	else if (width == 16)
		*(volatile uint16_t*)buf = (uint16_t)writeval;
	else
		*(volatile uint32_t*)buf = (uint32_t)writeval;

	unmap_sysmem(buf);

	return RET_CMD_DONE;
}

/*
 * Memory Read
 * - mr addr [width]
 */
static int do_mr(struct tiny_cmd *cmd, int argc, char *argv[])
{
	unsigned long addr = 0, width = 32;
	void *buf;

	addr = tc_strtoul(argv[1], NULL, 16);

	if (argc == 3)
		width = tc_strtoul(argv[2], NULL, 10);

	if (width != 8 && width != 16 && width != 32)
		return RET_CMD_USAGE;

	buf = map_sysmem(addr, 4);

	if (width == 8)
		tc_pr("0x%02x @ 0x%lx\n", *(uint8_t *)buf, addr);
	else if (width == 16)
		tc_pr("0x%04x @ 0x%lx\n", *(uint16_t *)buf, addr);
	else
		tc_pr("0x%08x @ 0x%lx\n", *(uint32_t *)buf, addr);

	unmap_sysmem(buf);

	return RET_CMD_DONE;
}

/*
 * Memory Display
 * - md addr len
 */
static int do_md(struct tiny_cmd *cmd, int argc, char *argv[])
{
	unsigned long addr = 0, length = 1, bytes = 0;
	unsigned int *buf;
	int i = 0;

	addr = tc_strtoul(argv[1], NULL, 16);
	length = tc_strtoul(argv[2], NULL, 16);

	bytes = length << 2;
	buf = (unsigned int *)map_sysmem(addr, bytes);

	for (i = 0; i < length; i++) {
		tc_pr("0x%08x @ 0x%lx\n", *buf, (unsigned long)buf);
		buf++;
	}

	unmap_sysmem(buf);

	return RET_CMD_DONE;
}


/*
 * Jump and run
 */
static int do_go(struct tiny_cmd *cmd, int argc, char *argv[])
{
	unsigned long addr = 0;
	func_entry payload_entry = NULL;

	addr = tc_strtoul(argv[1], NULL, 16);

	payload_entry = (func_entry)addr;

	tc_pr("Goodbye boot console, Jump to 0x%lx to run\n", addr);

	payload_entry();
	/* TODO: invalidate icache */

	return RET_CMD_DONE;
}


/*
 * Memory compare by byte.
 */
static int do_cmp(struct tiny_cmd *cmd, int argc, char *argv[])
{
	unsigned long addr0 = 0, addr1 = 0;
	unsigned char *buf0 = NULL, *buf1 = NULL;
	unsigned int bytes = 0, i = 0;

	addr0 = tc_strtoul(argv[1], NULL, 16);
	addr1 = tc_strtoul(argv[2], NULL, 16);
	bytes = tc_strtoul(argv[3], NULL, 16);

	buf0 = (unsigned char *)map_sysmem(addr0, bytes);
	buf1 = (unsigned char *)map_sysmem(addr1, bytes);

	for (i = 0; i < bytes; i++) {
		if (*buf0 != *buf1)
			break;
		buf0++;
		buf1++;
	}

	if (i == bytes)
		tc_pr("0x%lx is same as 0x%lx within %d bytes.\n",
			addr0, addr1, bytes);
	else
		tc_pr("Difference between 0x%lx(0x%02x) and 0x%lx(0x%02x)\n",
			addr0 + i, *buf0, addr1 + i, *buf1);

	return RET_CMD_DONE;
}

struct tiny_cmd *find_cmd(const char *name)
{
	int i = 0;

	for (i = 0; i < TC_CMD_TB_LEN; i++) {
		if (!strncmp(name, tc_cmd_tb[i].name,
				strlen(tc_cmd_tb[i].name)))
			break;
	}

	if (i < TC_CMD_TB_LEN)
		return &tc_cmd_tb[i];

	return NULL;
}

static const char *parse_int(const char *s, unsigned int *base)
{
	if (*base == 0) {
		if (s[0] == '0') {
			if (tc_tolower(s[1]) == 'x' && tc_isxdigit(s[2]))
				*base = 16;
			else
				*base = 8;
		} else {
			*base = 10;
		}
	}

	if (*base == 16 && s[0] == '0' && tc_tolower(s[1]) == 'x')
		s += 2;

	return s;
}

static unsigned long tc_strtoul(const char *cp, char **endp,
				unsigned int base)
{
	unsigned long result = 0;
	unsigned long value;

	cp = parse_int(cp, &base);

	while (tc_isxdigit(*cp) &&
		(value = tc_isdigit(*cp) ? *cp-'0' : (tc_islower(*cp)
		? tc_toupper(*cp) : *cp)-'A'+10) < base) {
		result = result*base + value;
		cp++;
	}

	if (endp)
		*endp = (char *)cp;

	return result;
}

static void *map_sysmem(unsigned long vaddr, unsigned long len)
{
	/* Return physical address*/
	return (void *)vaddr;
}

static void unmap_sysmem(void *vaddr)
{
	/* TODO: Do some resource release is necessary */
}

/*
 * Help
 */
extern void main_blinky( void );
static int do_run_blink(struct tiny_cmd *cmd, int argc, char *argv[])
{

	main_blinky();
	return RET_CMD_DONE;
}
