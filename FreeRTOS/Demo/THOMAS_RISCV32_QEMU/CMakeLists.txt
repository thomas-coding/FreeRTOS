cmake_minimum_required(VERSION 3.10)

project(thomas_riscv32)

#toolchain
set(CROSS_COMPILE $ENV{CROSS_COMPILE})
set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMIPLE}g++)
set(CMAKE_LD ${CROSS_COMPILE}ld)

set(CURRENT_FOLDER .)
set(FreeRTOS_TOP "${CURRENT_FOLDER}/../..")
set(FreeRTOS_Kernel "${FreeRTOS_TOP}/Source")
set(FreeRTOS_Common "${FreeRTOS_TOP}/Demo/Common")
set(FreeRTOS_PLUS "${FreeRTOS_TOP}/../FreeRTOS-Plus")


MESSAGE("FreeRTOS_KERNEL_PORT_DIR ${FreeRTOS_KERNEL_PORT_DIR}")

# Define linker and target
set(LD_SCRIPT "thomas_riscv32.lds")
set(Target "thomas_riscv32")

#compile option
set(CMAKE_C_FLAGS
  "-DportasmHANDLE_INTERRUPT=handle_trap -fmessage-length=0 \
  -march=rv32imac -mabi=ilp32 -mcmodel=medlow -ffunction-sections -fdata-sections \
  --specs=nano.specs -fno-builtin-printf  -Wno-unused-parameter -nostartfiles -g3 -Os ")

#link
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_HOME_DIRECTORY}/scripts/${LD_SCRIPT} \
    -march=rv32imac -mabi=ilp32 -mcmodel=medlow \
    -Wl,-Map=linkmap.txt,--build-id=none -nostartfiles -Xlinker --gc-sections \
    -Xlinker --defsym=__stack_size=350 ")

#include
include_directories(.)

#include bsp
include_directories(./bsp/board/ns16550)
include_directories(./bsp/board/timer)
include_directories(./bsp/driver/ns16550)
include_directories(./bsp/driver/plic)
include_directories(./bsp/driver/aclint)
include_directories(./bsp/board/interrupt)
include_directories(./bsp/common)
include_directories(./bsp/core)

#include portable
include_directories($(FreeRTOS_KERNEL_PORT_DIR))
include_directories(${FreeRTOS_Kernel}/portable/GCC/RISC-V)
include_directories(${FreeRTOS_Kernel}/portable/GCC/RISC-V/chip_specific_extensions/RV32I_CLINT_no_extensions)

#include freertos
include_directories(${FreeRTOS_Kernel}/include)
include_directories(${FreeRTOS_TOP}/Demo/Common/include)

#common source
aux_source_directory(. SRC_LIST)

set(BSP_SRC_LIST
    ${CURRENT_FOLDER}/bsp/board/ns16550/board-ns16550.c
    ${CURRENT_FOLDER}/bsp/board/interrupt/interrupt.c
    ${CURRENT_FOLDER}/bsp/board/timer/timer.c
    ${CURRENT_FOLDER}/bsp/driver/ns16550/ns16550.c
    ${CURRENT_FOLDER}/bsp/driver/plic/plic.c
    ${CURRENT_FOLDER}/bsp/driver/aclint/aclint.c
    ${CURRENT_FOLDER}/bsp/common/devices.c
    ${CURRENT_FOLDER}/bsp/common/console.c
    ${CURRENT_FOLDER}/bsp/core/riscv_cpu.c
    ${CURRENT_FOLDER}/bsp/core/start.S
    ${CURRENT_FOLDER}/bsp/core/vector.S
    )

set(FREERTOS_PORTABLE_SRC_LIST
    ${FreeRTOS_Kernel}/portable/GCC/RISC-V/port.c
    ${FreeRTOS_Kernel}/portable/GCC/RISC-V/portASM.S
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
    ${FREERTOS_SRC_LIST}
    ${FREERTOS_PORTABLE_SRC_LIST}
    )

set_property(
    SOURCE ${CURRENT_FOLDER}/bsp/core/start.S
    SOURCE ${CURRENT_FOLDER}/bsp/core/vector.S
    SOURCE ${FreeRTOS_Kernel}/portable/GCC/RISC-V/portASM.S
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
