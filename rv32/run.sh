#!/usr/bin/env bash
# Build the RV32I program for one input state and run it on Ripes.
#   [VARIANT=A0|A3] rv32/run.sh STATE [EXPECTED_LENGTH] [PROC]
# VARIANT selects the search: A0 (default, ida.h) or A3 (ida_perimeter.h).
# Prints the program output and the retired-instruction count.
# Build products go to rv32/build/ (ignored by Git).
set -euo pipefail
RIPES=${RIPES:-/Applications/Ripes-v2.2.6-106-g5b8a616-mac-universal2.app/Contents/MacOS/Ripes}
CC=${RVCC:-riscv64-elf-gcc}
CFLAGS="-O2 -march=rv32i -mabi=ilp32 -ffreestanding -nostdlib -static"
here=$(cd "$(dirname "$0")" && pwd)
state=$1 expect=${2:-} proc=${3:-RV32_ISS}
variant=${VARIANT:-A0}
case $variant in
A0) extra= ;;
A3) extra=-DPERIMETER ;;
*) echo "unknown VARIANT $variant" >&2; exit 2 ;;
esac

make -s -C "$here/.." tables.h perimeter.h
mkdir -p "$here/build"
elf="$here/build/$variant-$state.elf"
"$CC" $CFLAGS $extra -DSTATE="\"$state\"" ${expect:+-DEXPECT=$expect} \
    "$here/crt0.S" "$here/main.c" -o "$elf"
"$RIPES" --mode cli -t elf --proc "$proc" --iret --src "$elf"
