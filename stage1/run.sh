#!/usr/bin/env bash
# Stage 1 measurements on the pinned Ripes build.
#   ./run.sh rate <proc> <iters>   retired instructions, model time, wall time
#   ./run.sh mem  <bytes>          retired instructions, model time, peak host RSS
# Each run prints one tab-separated line so repeated runs can be tabulated.
set -euo pipefail
RIPES=${RIPES:-/Applications/Ripes-v2.2.6-106-g5b8a616-mac-universal2.app/Contents/MacOS/Ripes}
cd "$(dirname "$0")"

pp() { riscv64-elf-gcc -E -P -x assembler-with-cpp "$@"; }

# Run Ripes on the preprocessed program read from stdin under /usr/bin/time -l,
# which reports wall time and peak resident set size (bytes) on macOS.
run() {
    /usr/bin/time -l "$RIPES" --mode cli -t asm --src /dev/stdin "$@" 2>&1
}

field_after() { awk -v h="$1" '$0 ~ h {getline; print; exit}'; }

case "${1:-}" in
rate)
    proc=$2 iters=$3
    out=$(pp -DITERS="$iters" rate.S | run --proc "$proc" --iret --exectime)
    iret=$(field_after 'instructions retired' <<<"$out")
    ms=$(field_after 'execution time' <<<"$out")
    wall=$(awk '/ real /{print $1}' <<<"$out")
    printf 'rate\t%s\t%s\t%s\t%s\t%s\n' "$proc" "$iters" "$iret" "$ms" "$wall"
    ;;
mem)
    bytes=$2
    out=$(pp -DBYTES="$bytes" mem.S | run --proc RV32_ISS --iret --exectime)
    iret=$(field_after 'instructions retired' <<<"$out")
    ms=$(field_after 'execution time' <<<"$out")
    rss=$(awk '/maximum resident set size/{print $1}' <<<"$out")
    printf 'mem\t%s\t%s\t%s\t%s\n' "$bytes" "$iret" "$ms" "$rss"
    ;;
*)
    sed -n '2,5p' "$0"
    exit 1
    ;;
esac
