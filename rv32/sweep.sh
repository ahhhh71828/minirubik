#!/usr/bin/env bash
# Run the RV32I program on Ripes for every distance-11 state.
#   [VARIANT=A0|A3T|A3D|A3B|ASM] rv32/sweep.sh [JOBS]  > results.tsv
# VARIANT=ASM runs the hand-written rv32/solver.S (rv32/asm.sh); the others
# run the compiled C (rv32/run.sh).
# Each output line: STATE, retired instructions, OK|FAIL, and the printed
# moves, tab separated; awk -F'\t' '{print $1 "|" $4}' turns it into input
# for ./verify --solutions.
# The state list comes from ./ida --hardest (host), so the sweep covers all
# 2,644 distance-11 states; JOBS Ripes processes run in parallel.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
jobs=${1:-8}

make -s -C "$here/.." ida tables.h perimeter.h rv32/build/tables.inc >&2

one() {
    local state=$1 out iret status moves
    if [ "${VARIANT:-A0}" = ASM ]; then
        out=$("$here/asm.sh" "$state" 11 2>&1 | tr -d '\000') || true
    else
        out=$("$here/run.sh" "$state" 11 2>&1 | tr -d '\000') || true
    fi
    moves=$(head -1 <<<"$out" | tr -s ' ' | sed 's/^ //; s/ $//')
    iret=$(awk '/instructions retired/{getline; print; exit}' <<<"$out")
    if grep -q '^ *OK length' <<<"$out"; then status=OK; else status=FAIL; fi
    printf '%s\t%s\t%s\t%s\n' "$state" "${iret:-NA}" "$status" "$moves"
    rm -f "$here/build/"*"-$state.elf" "$here/build/"*"-$state.s"
}
export -f one
export here VARIANT

"$here/../ida" --hardest | awk '!/^#/{print $1}' |
    xargs -P "$jobs" -I{} bash -c 'one "$@"' _ {}
