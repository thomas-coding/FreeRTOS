/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef THOMAS_TEST_DEVICE_H
#define THOMAS_TEST_DEVICE_H

typedef struct TT_DEVICE_t {
    volatile unsigned int REG0;
    volatile unsigned int REG1;
    volatile unsigned int REG2;
    volatile unsigned int REG3;
    volatile unsigned int REG4;
} TT_DEVICE_t;

#define TT_DEVICE_ADDR ((TT_DEVICE_t *)(0x50000000))

void thomas_test_device_init(void);

#endif