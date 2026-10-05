# Stage 1: retired-instruction rate of a Ripes processor model.
# A memory loop of ITERS iterations, 4 instructions per iteration
# (store, two address/counter updates, branch).
# Preprocess with: riscv64-elf-gcc -E -P -x assembler-with-cpp -DITERS=<n>

.text
    li   t0, 0x10000000      # store target (data segment start)
    li   t1, ITERS           # remaining iterations
loop:
    sw   t1, 0(t0)           # same word every time: memory footprint stays fixed
    addi t1, t1, -1
    addi t2, t2, 1
    bne  t1, x0, loop
    li   a7, 10              # exit
    ecall
