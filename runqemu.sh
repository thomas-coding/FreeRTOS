#!/bin/bash

# shell folder
shell_folder=$(cd "$(dirname "$0")" || exit;pwd)
workspace_dir=${shell_folder}/../../..

qemu_version=qemu-6.0.0

# which demo to compile
demo_dir=${shell_folder}/FreeRTOS/Demo/THOMAS_M3_QEMU

# toolchain
qemu_try_dir1=${workspace_dir}/software/qemu
qemu_try_dir2=~/.toolchain/qemu

# get toolchain and export
function get_and_export_qemu()
{
    echo "get qemu"
    if [[ -d ${qemu_try_dir1} ]]; then
        if [[ -d ${qemu_try_dir1}/${qemu_version}/build ]]; then
            echo "find qemu from ${qemu_try_dir1}"
            export PATH="${qemu_try_dir1}/${qemu_version}/build:$PATH"
            return
        fi
    fi

    if [[ -d ${qemu_try_dir2} ]]; then
        if [[ -d ${qemu_try_dir2}/${qemu_version}/build ]]; then
            echo "find qemu from ${qemu_try_dir2}"
            export PATH="${qemu_try_dir2}/${qemu_version}/build:$PATH"
            return
        fi
    fi
    
    echo "cann't find qemu, please run root script './build.sh --qemu' ,exit ...."
    exit
}

#get_and_export_qemu

qemu_option=
if [[ $1  = "--gdb" ]]; then
    qemu_option+=" -s -S"
    echo "enable gdb, please run script './rungdb', and enter c "
else
    echo "not use gdb, just run"
fi

qemu_option+=" -machine thomas-m3"
qemu_option+=" -kernel ${demo_dir}/build/RTOSDemo.axf"
qemu_option+=" -serial stdio"

# run qemu
../qemu/build/qemu-system-arm ${qemu_option}

