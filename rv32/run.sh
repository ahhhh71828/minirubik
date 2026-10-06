#!/usr/bin/env bash
# Build the RV32I program for one input state and run it on Ripes.
#   [VARIANT=A0|A3T|A3D|A3B|A3F|A3BX|A3FX] rv32/run.sh STATE [EXPECTED_LENGTH] [PROC]
# VARIANT selects the search: A0 (default, ida.h) or an A3 variant of
# ida_perimeter.h (A3 is accepted as A3T, the stored-tail version); the X
# variants use the byte-offset tables of the hand-written solver.
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
A3 | A3T) variant=A3T extra="-DPERIMETER -DPERIMETER_TAIL" ;;
A3D) extra=-DPERIMETER ;;
A3B) extra="-DPERIMETER -DPERIMETER_BUCKET" ;;
A3F) extra="-DPERIMETER -DPERIMETER_BUCKET -DPERIMETER_LAZY" ;;
A3BX) extra="-DPERIMETER -DPERIMETER_BUCKET -DBYTE_TABLES" ;;
A3FX) extra="-DPERIMETER -DPERIMETER_BUCKET -DPERIMETER_LAZY -DBYTE_TABLES" ;;
*) echo "unknown VARIANT $variant" >&2; exit 2 ;;
esac

make -s -C "$here/.." tables.h perimeter.h tables_bytes.h
mkdir -p "$here/build"
elf="$here/build/$variant-$state.elf"
"$CC" $CFLAGS $extra -DSTATE="\"$state\"" ${expect:+-DEXPECT=$expect} \
    "$here/crt0.S" "$here/main.c" -o "$elf"
"$RIPES" --mode cli -t elf --proc "$proc" --iret --src "$elf"
