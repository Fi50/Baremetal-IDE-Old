#!/bin/bash
BAREMETAL_IDE_PATH=~/Downloads/Baremetal-IDE-BWRC
GDB=riscv64-unknown-elf-gdb

${GDB} ${BAREMETAL_IDE_PATH}/build/firmware.elf -x ${BAREMETAL_IDE_PATH}/scripts/program_dut.gdb 