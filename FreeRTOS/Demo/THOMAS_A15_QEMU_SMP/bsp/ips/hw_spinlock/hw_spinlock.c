/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <hw_spinlock.h>
#include <console.h>
#include <interrupt.h>

struct spin_lock hw_spin_lock[HW_SPIN_LOCK_NUMBER];

void hw_spin_lock_init(void) {
	for(int i = 0; i < HW_SPIN_LOCK_NUMBER; i++) {
		hw_spin_lock[i].id = i;
		hw_spin_lock[i].address = SPIN_LOCK_BASE + 4 * i;
		hw_spin_lock[i].recursive = 0;
		hw_spin_lock[i].used = 0;
	}
}

int hwspin_lock_request(void) {
	for(int i = 0; i < HW_SPIN_LOCK_NUMBER; i++) {
        if(hw_spin_lock[i].used == 0) {
            hw_spin_lock[i].used = 1;
            return i;
        }
	}
    return -1;
}

void hwspin_lock_free(int id) {
	for(int i = 0; i < HW_SPIN_LOCK_NUMBER; i++) {
        if(hw_spin_lock[i].id == id) {
            hw_spin_lock[i].used = 0;
        }
	}
}

/* used lock */
void hwspin_lock(int spin_lock_id) {
	/* spin for get lock */
	while(read_mreg32(hw_spin_lock[spin_lock_id].address) ==0) {};
}

void hwspin_unlock(int spin_lock_id) {
	write_mreg32(hw_spin_lock[spin_lock_id].address, 0);//write for unlock
}

void hwspin_lock_core_recursive(int spin_lock_id) {
	int cpuid = cpu_id_get();
	if(read_mreg32(hw_spin_lock[spin_lock_id].address) == 1) {//unlock
		hw_spin_lock[spin_lock_id].cpuid = cpuid;
		hw_spin_lock[spin_lock_id].recursive++;
		return;
	}

	if(hw_spin_lock[spin_lock_id].cpuid == cpuid) {//we ownerd self
		hw_spin_lock[spin_lock_id].recursive++;
		return;
	}

	/* spin for get lock */
	while(read_mreg32(hw_spin_lock[spin_lock_id].address) ==0) {};

	/* get lock, update cpuid and recursive */
	hw_spin_lock[spin_lock_id].cpuid = cpuid;
	hw_spin_lock[spin_lock_id].recursive++;
}

void hwspin_unlock_core_recursive(int spin_lock_id) {

	hw_spin_lock[spin_lock_id].recursive--;
	
	if(hw_spin_lock[spin_lock_id].recursive == 0) {
		write_mreg32(hw_spin_lock[spin_lock_id].address, 0);//write for unlock
	}

}