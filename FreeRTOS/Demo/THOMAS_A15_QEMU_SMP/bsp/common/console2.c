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
#include <hw_spinlock.h>

int printf_lock;

#define out_byte	uart_putc

/* This routine puts pad characters into the output buffer. */
static void padding(const int l_flag, struct params_s *par)
{
	uint32_t i = 0;

	if (par->do_padding && l_flag && (par->len < par->num_integer)) {
		for (i = par->len; i < par->num_integer; i++)
			out_byte(par->pad_character);
	}
}

/*
 * This routine moves a string to the output buffer as directed by the padding
 * and positioning flags.
 */
static void out_str(char *lp, struct params_s *par)
{
	/* pad on left if needed */
	par->len = strlen(lp);
	padding(!(par->left_flag), par);

	/* Move string to the buffer */
	while (*lp && (par->num_decimal)--)
		out_byte(*lp++);

	/* Pad on right if needed */
	/* CR 439175 - elided next stmt. Seemed bogus. */
	padding(par->left_flag, par);
}

/*
 * This routine moves a number to the output buffer as directed by the padding
 * and positioning flags.
 */
static void out_num(const long n, const long base, struct params_s *par)
{
	char *cp = NULL;
	uint8_t negative = 0;
	char outbuf[32] = {0};
	char digits_upper[] = "0123456789ABCDEF";
	char digits_lower[] = "0123456789abcdef";
	char *digits = digits_lower;
	unsigned long num = 0;

	/* Check if number is negative */
	if (base == 10 && n < 0L) {
		negative = 1;
		num = -(n);
	} else {
		num = (n);
		negative = 0;
	}

	if (par->hex_upper)
		digits = digits_upper;

	/* Build number (backwards) in outbuf */
	cp = outbuf;
	do {
		*cp++ = digits[(int)(num % base)];
	} while ((num /= base) > 0);

	if (negative)
		*cp++ = '-';
	*cp-- = 0;

	/*
	 * Move the converted number to the buffer and add in the padding
	 * where needed.
	 */
	par->len = strlen(outbuf);
	padding(!(par->left_flag), par);
	while (cp >= outbuf)
		out_byte(*cp--);
	padding(par->left_flag, par);
}

/*
 * This routine moves a number to the output buffer as directed by the padding
 * and positioning flags.
 */
static void out_long_num(const unsigned long long n, const long base,
						struct params_s *par)
{
	char *cp = NULL;
	uint8_t negative = 0;
	char outbuf[64] = {0};
	char digits_upper[] = "0123456789ABCDEF";
	char digits_lower[] = "0123456789abcdef";
	char *digits = digits_lower;
	unsigned long long num = 0;

	/* Check if number is negative */
	if (base == 10 && n < 0L) {
		negative = 1;
		num = -(n);
	} else {
		num = (n);
		negative = 0;
	}

	if (par->hex_upper)
		digits = digits_upper;

	/* Build number (backwards) in outbuf */
	cp = outbuf;
	do {
		*cp++ = digits[(int)(num % base)];
	} while ((num /= base) > 0);

	if (negative)
		*cp++ = '-';
	*cp-- = 0;

	/*
	 * Move the converted number to the buffer and add in the padding
	 * where needed.
	 */
	par->len = strlen(outbuf);
	padding(!(par->left_flag), par);
	while (cp >= outbuf)
		out_byte(*cp--);
	padding(par->left_flag, par);
}

/* This routine gets a number from the format string. */
static int get_num(char **linep)
{
	int n = 0;
	char *cp = NULL;

	n = 0;
	cp = *linep;
	while (isdigit((unsigned char)*cp))
		n = n*10 + ((*cp++) - '0');
	*linep = cp;

	return n;
}

/*
 * This routine operates just like a printf/sprintf routine. It outputs a set
 * of data under the control of a formatting string. Not all of the standard
 * C format control are supported. The ones provided are primarily those needed
 * for embedded systems work. Primarily the floaing point routines are omitted.
 * Other formats could be added easily by following the examples shown for the
 * supported formats.
 */

void console_printf(const char *ctrl1, ...)
{
	uint8_t long_flag = 0;
	uint8_t long_long_flag = 0;
	uint8_t dot_flag = 0;
	uint8_t long_cnt = 0;

	struct params_s par = {0};

	char ch = 0;
	va_list argp;
	char *ctrl = (char *)ctrl1;

	hwspin_lock(printf_lock);

	va_start(argp, ctrl1);

	for (; *ctrl; ctrl++) {

		/*
		 * move format string chars to buffer until a format control
		 * is found.
		 */
		if (*ctrl != '%') {
			if (*ctrl != '\n') {
				out_byte(*ctrl);
			} else {
				out_byte('\r');
				out_byte('\n');
			}
			continue;
		}

		/* initialize all the flags for this format. */
		dot_flag = long_flag = par.left_flag = par.do_padding = 0;
		long_long_flag = 0;
		long_cnt = 0;
		par.pad_character = ' ';
		par.num_decimal = 32767;
		par.hex_upper = 0;

 try_next:
		ch = *(++ctrl);

		if (isdigit(ch)) {
			if (dot_flag) {
				par.num_decimal = get_num(&ctrl);
			} else {
				if (ch == '0')
					par.pad_character = '0';

				par.num_integer = get_num(&ctrl);
				par.do_padding = 1;
			}
			ctrl--;
			goto try_next;
		}

		switch (tolower(ch)) {
		case '%':
			out_byte('%');
			continue;
		case '-':
			par.left_flag = 1;
			break;
		case '.':
			dot_flag = 1;
			break;
		case 'l':
			long_flag = 1;
			long_cnt++;
			if (long_cnt == 2) {
				long_long_flag = 1;
				long_flag = 0;
			}
			break;
		case 'o':
			if (long_flag || ch == 'O') {
				out_num((long)va_arg(argp,
						 long), 8L, &par);
				continue;
			} else if (long_long_flag || ch == 'O') {
				out_long_num((long long)va_arg(argp,
						 long long), 8L, &par);
				continue;
			} else {
				out_num((int)va_arg(argp, int), 8L, &par);
				continue;
			}
		case 'd':
			if (long_flag || ch == 'D') {
				out_num((long)va_arg(argp,
						 long), 10L, &par);
				continue;
			} else if (long_long_flag || ch == 'D') {
				out_long_num((long long)va_arg(argp,
						 long long), 10L, &par);
				continue;
			} else {
				out_num((int)va_arg(argp, int), 10L, &par);
				continue;
			}
		case 'u':
			if (long_flag || ch == 'U') {
				out_num((unsigned long)va_arg(argp,
					 unsigned long), 10L, &par);
				continue;
			} else if (long_long_flag || ch == 'U') {
				out_long_num((unsigned long long)va_arg(argp,
					 unsigned long long), 10L, &par);
				continue;
			} else {
				out_num((uint32_t)va_arg(argp, uint32_t), 10L, &par);
				continue;
			}
		case 'x':
			if (ch == 'X')
				par.hex_upper = 1;

			if (long_flag) {
				out_num((unsigned long)va_arg(argp,
						unsigned long), 16L, &par);
				continue;
			} else if (long_long_flag) {
				out_long_num((unsigned long long)va_arg(argp,
					 unsigned long long), 16L, &par);
				continue;
			} else {
				out_num((uint32_t)va_arg(argp, uint32_t), 16L, &par);
				continue;
			}
		case 's':
			out_str(va_arg(argp, char *), &par);
			continue;
		case 'c':
			out_byte(va_arg(argp, int));
			continue;
		case '\\':
			switch (*ctrl) {
			case 'a':
				out_byte(0x07);
				break;
			case 'h':
				out_byte(0x08);
				break;
			case 'r':
				out_byte(0x0D);
				break;
			case 'n':
				out_byte('\r');
				out_byte('\n');
				break;
			default:
				out_byte(*ctrl);
				break;
			}
			ctrl++;
			break;
		default:
			continue;
		}
		goto try_next;
	}
	va_end(argp);

	hwspin_unlock(printf_lock);
}

void console2_init(void)
{
	uart_init();
	printf_lock = hwspin_lock_request();
}
