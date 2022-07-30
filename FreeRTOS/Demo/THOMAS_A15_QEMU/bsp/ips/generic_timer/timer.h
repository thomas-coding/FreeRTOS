/*
 * Generic driver code for timer
 *
 * Copyright (C) 2021 VeriSilicon Holdings Co., Ltd.
 *
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include "interrupt.h"
#include "generic_timer.h"

struct gtimer_ops {
	uint32_t  irq;
	char *name;

	void (*enable)(void);
	void (*disable)(void);

	void (*en_irq)(void);
	void (*dis_irq)(void);

	void (*ctl_set)(uint32_t);
	void (*tval_set)(uint32_t);
	void (*cval_set)(uint64_t);

	uint32_t (*ctl_get)(void);
	uint32_t (*tval_get)(void);
	uint64_t (*cval_get)(void);
	uint64_t (*tick_get)(void);
};

struct timer_operation {
	void (*timer_init)(void *timer_info, uint32_t mode, uint32_t timer_id);
	void (*timer_open)(void *timer_info, uint32_t timer_id);
	void (*timer_close)(void *timer_info, uint32_t timer_id);
	uint32_t (*timer_cur_val)(void *timer_info, uint32_t timer_id);
};

void gtimer_init(void);

extern struct gtimer_ops gt_ops;

#endif /* __TIMER_H__ */
