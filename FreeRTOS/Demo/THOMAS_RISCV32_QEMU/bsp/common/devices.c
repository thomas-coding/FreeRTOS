/*
 * Source for devices.
 *
 * Copyright (C) 2021 VeriSilicon Holdings Co., Ltd.
 *
 */

#include "common.h"
#include "devices.h"
#include "timer.h"
#include "interrupt.h"

void devices_init(void)
{
    interrupt_global_enable();
    console_init();
    timer_init();
    interrupt_init();
}
