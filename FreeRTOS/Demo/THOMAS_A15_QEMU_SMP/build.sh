#!/bin/bash

# shell folder
shell_folder=$(cd "$(dirname "$0")" || exit;pwd)

export PATH="/home/cn1396/.toolchain/gcc-arm-10.3-2021.07-x86_64-arm-none-eabi/bin/:$PATH"
export CROSS_COMPILE=arm-none-eabi-

# which demo to compile
demo_dir=${shell_folder}



# build
cd "${demo_dir}" || exit
rm -rf build
cmake -B build
make -C build
