# Stage 1: host memory consumed per guest byte written.
# Writes BYTES bytes of guest memory with one 32-bit store per word,
# each word at a fresh address. Comparing a large BYTES against a small
# control gives the host-bytes-per-guest-byte slope.
# Preprocess with: riscv64-elf-gcc -E -P -x assembler-with-cpp -DBYTES=<n>

.text
    li   t0, 0x10000000      # first guest address written
    li   t1, BYTES
    add  t1, t0, t1          # one past the last address
    li   t2, 0x01010101      # every byte nonzero, in case zero writes are elided
loop:
    sw   t2, 0(t0)
    addi t0, t0, 4
    bltu t0, t1, loop
    li   a7, 10              # exit
    ecall
