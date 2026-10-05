#!/usr/bin/env bash
# Build the RV32I A0 program for one input state and run it on Ripes.
#   rv32/run.sh STATE [EXPECTED_LENGTH] [PROC]
# Prints the program output and the retired-instruction count.
# Build products go to rv32/build/ (ignored by Git).
set -euo pipefail
RIPES=${RIPES:-/Applications/Ripes-v2.2.6-106-g5b8a616-mac-universal2.app/Contents/MacOS/Ripes}
CC=${RVCC:-riscv64-elf-gcc}
CFLAGS="-O2 -march=rv32i -mabi=ilp32 -ffreestanding -nostdlib -static"
here=$(cd "$(dirname "$0")" && pwd)
state=$1 expect=${2:-} proc=${3:-RV32_ISS}

make -s -C "$here/.." tables.h
mkdir -p "$here/build"
elf="$here/build/a0-$state.elf"
"$CC" $CFLAGS -DSTATE="\"$state\"" ${expect:+-DEXPECT=$expect} \
    "$here/crt0.S" "$here/main.c" -o "$elf"
"$RIPES" --mode cli -t elf --proc "$proc" --iret --src "$elf"
