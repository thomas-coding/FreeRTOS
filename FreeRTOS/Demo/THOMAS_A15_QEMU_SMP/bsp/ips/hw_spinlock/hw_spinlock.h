/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HW_SPINLOCK_H
#define HW_SPINLOCK_H

#include <stddef.h>
#include <stdint.h>

#define SPIN_LOCK_BASE	0x41000000
#define HW_SPIN_LOCK_NUMBER 4

struct spin_lock {
	int id;
    int used;
	uint32_t address;
	uint32_t recursive;
	int cpuid; //ownerd to which core
};

void hw_spin_lock_init(void);
int hwspin_lock_request(void);
void hwspin_lock_free(int id);
void hwspin_lock(int spin_lock_id);
void hwspin_unlock(int spin_lock_id);
void hwspin_lock_core_recursive(int spin_lock_id);
void hwspin_unlock_core_recursive(int spin_lock_id);

#endif