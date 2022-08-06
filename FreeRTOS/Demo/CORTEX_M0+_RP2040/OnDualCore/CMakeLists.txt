cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)

project(example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_library(on_dual_core_common INTERFACE)
target_sources(on_dual_core_common INTERFACE
        main.c)
target_include_directories(on_dual_core_common INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(on_dual_core_common INTERFACE
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap1
        pico_stdlib
        pico_multicore)

add_executable(on_dual_core)
target_link_libraries(on_dual_core on_dual_core_common)
pico_add_extra_outputs(on_dual_core)
pico_enable_stdio_usb(on_dual_core 1)
