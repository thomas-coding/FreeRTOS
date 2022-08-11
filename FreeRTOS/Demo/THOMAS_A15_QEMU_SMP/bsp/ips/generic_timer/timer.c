/*
 * Generic driver code for timer
 *
 * Copyright (C) 2021 VeriSilicon Holdings Co., Ltd.
 *
 */

#include "timer.h"

struct gtimer_ops gt_ops;

uint64_t gtimer_get_ticks(void)
{
	uint64_t ticks = 0;

	ticks = gtimer_get_cntp_ct();

	return ticks;
}

void gtimer_init(void)
{
	gt_ops.enable  = gtimer_enable;
	gt_ops.disable = gtimer_disable;

	gt_ops.en_irq  = gtimer_en_irq;
	gt_ops.dis_irq = gtimer_dis_irq;

	gt_ops.ctl_set  = gtimer_set_cntp_ctl;
	gt_ops.tval_set = gtimer_set_cntp_tval;
	gt_ops.cval_set = gtimer_set_cntp_cval;

	gt_ops.ctl_get  = gtimer_get_cntp_ctl;
	gt_ops.tval_get = gtimer_get_cntp_tval;
	gt_ops.cval_get = gtimer_get_cntp_cval;
	gt_ops.tick_get = gtimer_get_ticks;

#if (CONFIG_NUM_CPUS == 1) /* LPS */
	gtimer_set_clk();
#endif

#ifndef CONFIG_NON_SECURE
	gtimer_set_cntp_freq(PERIPH_CLK);
#endif
}
