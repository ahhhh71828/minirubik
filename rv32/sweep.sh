#!/usr/bin/env bash
# Run the RV32I program on Ripes for every distance-11 state.
#   [VARIANT=A0|A3] rv32/sweep.sh [JOBS]  > results.tsv
# Each output line: STATE <tab> retired instructions <tab> OK|FAIL.
# The state list comes from ./ida --hardest (host), so the sweep covers all
# 2,644 distance-11 states; JOBS Ripes processes run in parallel.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
jobs=${1:-8}

make -s -C "$here/.." ida tables.h perimeter.h >&2

one() {
    local state=$1 out iret status
    out=$("$here/run.sh" "$state" 11 2>&1) || true
    iret=$(awk '/instructions retired/{getline; print; exit}' <<<"$out")
    if grep -q '^ *OK length' <<<"$out"; then status=OK; else status=FAIL; fi
    printf '%s\t%s\t%s\n' "$state" "${iret:-NA}" "$status"
    rm -f "$here/build/${VARIANT:-A0}-$state.elf"
}
export -f one
export here VARIANT

"$here/../ida" --hardest | awk '!/^#/{print $1}' |
    xargs -P "$jobs" -I{} bash -c 'one "$@"' _ {}
