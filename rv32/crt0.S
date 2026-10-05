# Minimal start-up for the bare-metal RV32I build loaded by Ripes as ELF.
# Sets gp for linker relaxation and sp for the C stack, calls main, and
# exits through the Ripes exit ecall.

    .section .text.start
    .globl _start
_start:
    .option push
    .option norelax
    la   gp, __global_pointer$
    .option pop
    li   sp, 0x7ffffff0
    call main
    li   a7, 10
    ecall
