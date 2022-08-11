/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <thomas_test_device.h>
#include <console.h>
#include <interrupt.h>

/* QEMU Test device, REG2 = REG0 + REG1 */
/* It has irq , number 15 */

void thomas_test_device_reg_test(void)
{
    TT_DEVICE_ADDR->REG0 = 0x22;
    TT_DEVICE_ADDR->REG1 = 0x33;
    bm_printf_value_u32("thomas test device REG0: ", TT_DEVICE_ADDR->REG0);
    bm_printf_value_u32("thomas test device REG1: ", TT_DEVICE_ADDR->REG1);
    bm_printf_value_u32("thomas test device REG2: ", TT_DEVICE_ADDR->REG2);
}

/* GICV3 SPI irq */
#define THOMAS_TEST_DEVICET_IRQ (32 + 100)

void thomas_test_device_irq_handler(int irq, void *dev)
{
    bm_printf("Enter thomas_test_device_irq_handler!\n");
    /* clear interrupt */
    TT_DEVICE_ADDR->REG4 = 0xa5;
}

void thomas_test_device_irq_test(void)
{
    bm_printf("thomas_test_device_irq_test\n");

    request_irq_origin(THOMAS_TEST_DEVICET_IRQ, ISR_TYPE_IRQ, thomas_test_device_irq_handler, 0);
    /* trigger interrupt */
    gic_set_interrupt_pending(THOMAS_TEST_DEVICET_IRQ);
}

/**
 * Initializes
 */
void thomas_test_device_init(void)
{
    thomas_test_device_reg_test();
    thomas_test_device_irq_test();
}
