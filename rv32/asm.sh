#!/usr/bin/env bash
# Build the hand-written solver (rv32/solver.S) for one input and run it.
#   [HEURISTIC=A0] [DEFS='-DNAME ...'] [RENDER=1] [RENDER_DELAY=500000] [SIZE=1]
#   rv32/asm.sh STATE [EXPECTED_LENGTH] [PROC]
# DEFS passes extra assemble-time switches, e.g. DEFS=-DEAGER_LOOKUP.
#
# The C preprocessor resolves the assemble-time switches (Ripes has no .if),
# then Ripes' own assembler runs the result (-t asm): this is the measured
# path. SIZE=1 also assembles the same preprocessed text with GNU as, prints
# the section sizes, and runs that ELF too, so the two retired counts can be
# compared. RENDER=1 only writes the GUI source and stops (the CLI has no
# LED matrix). Products go to rv32/build/ (ignored by Git).
set -euo pipefail
RIPES=${RIPES:-/Applications/Ripes-v2.2.6-106-g5b8a616-mac-universal2.app/Contents/MacOS/Ripes}
CC=${RVCC:-riscv64-elf-gcc}
here=$(cd "$(dirname "$0")" && pwd)
state=$1 expect=${2:-} proc=${3:-RV32_ISS}
render=${RENDER:-0}
tag=asm${HEURISTIC:+-$HEURISTIC}

make -s -C "$here/.." rv32/build/tables.inc rv32/build/tables_bytes.inc
src="$here/build/$tag-$state.s"
[ "$render" != 0 ] && src="$here/build/$tag-$state-gui.s"
"$CC" -E -P -x assembler-with-cpp -I"$here/build" \
    -DSTATE="\"$state\"" ${expect:+-DEXPECT=$expect} -DRENDER="$render" ${RENDER_DELAY:+-DRENDER_DELAY=$RENDER_DELAY} \
    ${HEURISTIC:+-DHEURISTIC_$HEURISTIC} ${DEFS:-} "$here/solver.S" > "$src"

if [ "$render" != 0 ]; then
    echo "GUI source: $src"
    exit 0
fi
"$RIPES" --mode cli -t asm --proc "$proc" --iret --src "$src"

if [ -n "${SIZE:-}" ]; then
    elf="$here/build/$tag-$state.elf"
    "$CC" -march=rv32i -mabi=ilp32 -nostdlib -static -Wl,--no-relax \
        -x assembler "$src" -o "$elf"
    riscv64-elf-size -A "$elf" | awk '$1 ~ /^\.(text|data|bss|rodata|sdata|sbss)$/'
    echo "GNU as ELF on Ripes:"
    "$RIPES" --mode cli -t elf --proc "$proc" --iret --src "$elf" |
        awk '/instructions retired/{getline; print "  retired " $0}'
fi
