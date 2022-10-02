cmake_minimum_required(VERSION 3.10)

project(thomas_a15)

include(config.cmake)

#toolchain
set(CROSS_COMPILE $ENV{CROSS_COMPILE})
set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMIPLE}g++)
set(CMAKE_LD ${CROSS_COMPILE}ld)

set(CURRENT_FOLDER .)
set(FreeRTOS_TOP "${CURRENT_FOLDER}/../..")
set(FreeRTOS_Kernel "${FreeRTOS_TOP}/Source")
set(FreeRTOS_Common ${FreeRTOS_TOP}/Demo/Common)
set(FreeRTOS_PLUS "${FreeRTOS_TOP}/../FreeRTOS-Plus")

# Define linker and target
set(LD_SCRIPT "thomas_a15.lds")
set(Target "thomas_a15")

#compile option
set(CMAKE_C_FLAGS
  "-mcpu=cortex-a15 -mfloat-abi=hard -mfpu=neon-vfpv4 \
  -Wall -fno-builtin -g -Wl,--print-memory-usage \
  -DmainCREATE_SIMPLE_BLINKY_DEMO_ONLY=1 ")

#link
#--specs=rdimon.specs
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_HOME_DIRECTORY}/scripts/${LD_SCRIPT} \
-Wl,-Map=linkmap.txt,--build-id=none -nostartfiles \
--specs=nosys.specs ")

#include
include_directories(.)

#include portable
include_directories(./portable)

#include bsp
include_directories(./bsp)
include_directories(./bsp/ips/cache)
include_directories(./bsp/ips/generic_timer)
include_directories(./bsp/ips/gic/gicv3)
include_directories(./bsp/ips/misc)
include_directories(./bsp/ips/mmu)
include_directories(./bsp/ips/pl011)
include_directories(./bsp/ips/smp)
include_directories(./bsp/ips/ttdevice)
include_directories(./bsp/common)
include_directories(./bsp/core)

#include freertos
include_directories(${FreeRTOS_Kernel}/include)
include_directories(${FreeRTOS_TOP}/Demo/Common/include)

#common source
aux_source_directory(. SRC_LIST)

set(BSP_SRC_LIST
    ${CURRENT_FOLDER}/bsp/platform.c
    ${CURRENT_FOLDER}/bsp/core/start.S
    ${CURRENT_FOLDER}/bsp/ips/cache/cache_asm.S
    ${CURRENT_FOLDER}/bsp/ips/cache/cache.c
    ${CURRENT_FOLDER}/bsp/ips/generic_timer/generic_timer.c
    ${CURRENT_FOLDER}/bsp/ips/generic_timer/timer.c
    ${CURRENT_FOLDER}/bsp/ips/gic/gicv3/a32.S
    ${CURRENT_FOLDER}/bsp/ips/gic/gicv3/gicv3-gicd.c
    ${CURRENT_FOLDER}/bsp/ips/gic/gicv3/gicv3-gicr.c
    ${CURRENT_FOLDER}/bsp/ips/gic/gicv3/interrupt.c
    ${CURRENT_FOLDER}/bsp/ips/misc/misc.c
    ${CURRENT_FOLDER}/bsp/ips/mmu/mmu.c
    ${CURRENT_FOLDER}/bsp/ips/pl011/pl011.c
    ${CURRENT_FOLDER}/bsp/ips/smp/smp.c
    ${CURRENT_FOLDER}/bsp/ips/ttdevice/thomas_test_device.c
    ${CURRENT_FOLDER}/bsp/common/console.c
    ${CURRENT_FOLDER}/bsp/common/console2.c
    ${CURRENT_FOLDER}/bsp/common/tiny_console.c
    ${CURRENT_FOLDER}/bsp/common/tiny_command.c
    )

if(${CONFIG_LIBC_STUB})
set(LIBC_STUB_SRC_LIST
    ${CURRENT_FOLDER}/libcstub/write.c
    ${CURRENT_FOLDER}/libcstub/malloc.c
    )
add_definitions(-DCONFIG_LIBC_STUB)
endif()

if(${CONFIG_SMP})
add_definitions(-DCONFIG_SMP)
endif()

if(${CONFIG_TINY_CONSOLE})
add_definitions(-DCONFIG_TINY_CONSOLE)
endif()

set(FREERTOS_PORTABLE_SRC_LIST
    ${CURRENT_FOLDER}/portable/port.c
    ${CURRENT_FOLDER}/portable/portASM.S
    )

set(FREERTOS_SRC_LIST
    ${FreeRTOS_Kernel}/tasks.c
    ${FreeRTOS_Kernel}/timers.c
    ${FreeRTOS_Kernel}/list.c
    ${FreeRTOS_Kernel}/queue.c
    ${FreeRTOS_Kernel}/event_groups.c
    ${FreeRTOS_Kernel}/portable/MemMang/heap_4.c
    )

set(SRC_LIST
    ${SRC_LIST}
    ${BSP_SRC_LIST}
    ${LIBC_STUB_SRC_LIST}
    ${FREERTOS_SRC_LIST}
    ${FREERTOS_PORTABLE_SRC_LIST}
    )

set_property(
    SOURCE ${CURRENT_FOLDER}/bsp/core/start.S
    SOURCE ${CURRENT_FOLDER}//bsp/ips/cache/cache_asm.S
    SOURCE ${CURRENT_FOLDER}//bsp/ips/gic/gicv3/a32.S
    SOURCE ${CURRENT_FOLDER}/portable/portASM.S
    PROPERTY LANGUAGE C
)

link_libraries(m)
add_executable(${Target}.elf "")

#target
target_sources(${Target}.elf
    PRIVATE
    ${SRC_LIST}
)

add_custom_target(${Target}.bin ALL
    COMMAND ${CROSS_COMPILE}objcopy -O binary ${Target}.elf ${Target}.bin
    COMMAND ${CROSS_COMPILE}objdump -xd ${Target}.elf > ${Target}.asm
    DEPENDS ${Target}.elf
)
