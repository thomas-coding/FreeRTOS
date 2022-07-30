/*
 * Source for platform.
 *
 * Copyright (C) 2022 VeriSilicon Holdings Co., Ltd.
 *
 */
#include "platform.h"

#include <console.h>
#include <mmu.h>
#include <cache.h>
#include <interrupt.h>
#include <misc.h>
#include <generic_timer.h>
#include <timer.h>
#include <thomas_test_device.h>
#include <smp.h>

void platform_init(void)
{
	console_init();
	mmu_init();
	cache_init();
	gic_init();
	//misc_init();
	//generic_timer_init();
    gtimer_init();
	//thomas_test_device_init();
	//smp_init();
}
