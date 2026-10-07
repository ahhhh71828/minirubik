/* minirubik A3B solver, hand-written RV32I assembly (asm v2b).
 *
 * Search: IDA* over (permutation rank, orientation rank). Heuristic:
 * h0 = max(perm_pdb[p], ori_pdb[o]); when h0 <= PERIMETER_RADIUS the
 * bucketed perimeter gives the exact distance, or PERIMETER_RADIUS + 1 if
 * the state is absent. -DHEURISTIC_A0 drops the perimeter (A0 fallback).
 * -DEAGER_LOOKUP looks up whenever h0 is within the radius, as the C A3B
 * does, instead of only when few moves are left (measurement switch).
 * -DCALL_LOOKUP keeps the asm v1 subroutine call for the in-search lookup.
 * -DINDEX_TABLES keeps plain ranks in the tables (asm v1, v2a).
 *
 * Build: preprocess with the C preprocessor, then assemble with Ripes
 * (-t asm) or GNU as. The source uses only what both assemblers accept;
 * Ripes has no .if, so the assemble-time switches are #if:
 *   riscv64-elf-gcc -E -P -x assembler-with-cpp -Irv32/build \
 *       -DSTATE='"21345671111111"' [-DEXPECT=11] [-DRENDER=0] rv32/solver.S
 * rv32/asm.sh wraps this. Tables come from gen_tables --asm.
 *
 * Search registers (the current depth lives in registers; a 32-byte frame
 * per depth holds only what backtracking needs):
 *   s0  perm_turn        s1  ori_turn         s2  perm_pdb     s3  ori_pdb
 *   s4  perimeter_offset s5  perimeter_entry  s6  bound
 *   s7  frame of the current depth            s8  frame of depth 0
 *   s9  perm_turn row of the current face     s10 ori_turn row of it
 *   s11 PERM_FACE_BYTES, the distance between perm_turn rows
 *   a0  face being tried (0 R, 1 B, 2 D)      a1  quarter turns so far - 1
 *   a2  child perm rank  a3  child ori rank   a4  node perm  a5  node ori
 *   a6  moves left within the bound below the child: bound - (depth + 1)
 *   a7  face that led to this node (3 at the root)
 *   gp  root perm rank   tp  root ori rank
 * Frame: 0 s9, 4 s10, 8 node p, 10 node o, 12 child p, 14 child o,
 *        16 face, 17 turns. The sentinel frame before depth 0 has face 3.
 */
#ifndef STATE
#define STATE "21345671111111"
#endif

/* Table layout. Default (asm v2b): every rank is kept pre-scaled by 2, the
 * byte offset of its halfword entry, so a lookup is add + load with no
 * shift; the pattern databases are widened to halfwords so the same scaled
 * rank indexes them. -DINDEX_TABLES restores plain ranks (asm v1, v2a). */
#ifdef INDEX_TABLES
#define LOAD_PDB lbu
#define ORI_KEY_SHIFT 3             /* entry key ori << 3 from ori */
#else
#define LOAD_PDB lhu
#define ORI_KEY_SHIFT 2             /* entry key ori << 3 from ori * 2 */
#endif
#ifndef RENDER
#define RENDER 0
#endif
#define FRAME 32

    .data
/* Word data first: the data segment start is word aligned. */
frame_sentinel:
    .zero 16
    .byte 3
    .zero 15
frames:
    .zero 384                   /* 12 frames, depth 0 to 11 */
#ifdef INDEX_TABLES
#include "tables.inc"           /* halfword tables, then byte tables */
#else
#include "tables_bytes.inc"     /* every table halfword, ranks scaled by 2 */
#endif
input:
    .string STATE
digits:
    .zero 7                     /* permutation digits of the input */
counts:
    .zero 6                     /* counts[0..5] for the permutation rank */
moves:
    .zero 11                    /* solution, move = face * 3 + turns */
names:                          /* 4 bytes per move name */
    .byte 82, 0, 0, 0,  82, 50, 0, 0,  82, 39, 0, 0
    .byte 66, 0, 0, 0,  66, 50, 0, 0,  66, 39, 0, 0
    .byte 68, 0, 0, 0,  68, 50, 0, 0,  68, 39, 0, 0
msg_invalid:
    .string "FAIL: invalid state\n"
msg_ok:
    .string "OK length "
msg_fail:
    .string "FAIL length "
msg_newline:
    .string "\n"

#if RENDER
/* Face IDs: U=0, R=1, F=2, D=3, L=4, B=5. Each corner's three slots
 * are ordered so color(slot) = basis[cubie][(slot + twist) % 3].
 * The ordered triples preserve the baseline's +1/+2 twist convention. */
    .align 2
render_palette:
    .word 0x00ffffff, 0x00ff0000, 0x0000b050
    .word 0x00ffff00, 0x00ff8000, 0x000055ff
render_basis:
    .byte 0,2,4, 0,1,2, 3,2,1, 3,4,2
    .byte 0,5,1, 3,1,5, 3,5,4, 0,4,5
render_source:                  /* destination 1..7 takes source 1..7 */
    .byte 2,5,3,1,4,6,7
    .byte 1,2,3,5,6,7,4
    .byte 1,3,6,4,2,5,7
render_twist:
    .byte 1,2,0,2,1,0,0
    .byte 0,0,0,1,2,1,2
    .byte 0,0,0,0,0,0,0
render_perm:
    .zero 8
render_ori:
    .zero 8
render_next_perm:
    .zero 8
render_next_ori:
    .zero 8
/* Records: corner position, slot in render_basis[position], pixel x,y.
 * Net: U above F; L F R B across; D below F. Facelets are 4x3 pixels.
 * Faces start at x=0/9/18/27, y=2/9/16. Separators stay black. */
render_facelets:
    .byte 7,0,9,2,   4,0,13,2,  0,0,9,5,   1,0,13,5
    .byte 7,1,0,9,   0,2,4,9,   6,2,0,12,  3,1,4,12
    .byte 0,1,9,9,   1,2,13,9,  3,2,9,12,  2,1,13,12
    .byte 1,1,18,9,  4,2,22,9,  2,2,18,12, 5,1,22,12
    .byte 4,1,27,9,  7,2,31,9,  5,2,27,12, 6,1,31,12
    .byte 3,0,9,16,  2,0,13,16, 6,0,9,19,  5,0,13,19
#ifndef RENDER_DELAY
#define RENDER_DELAY 500000
#endif
#endif

    .text
    .globl _start
_start:
/* ---- Parse the 14-character input and validate it. -------------------
 * t0 input cursor, t1 digit cursor, t2 index i, t3 cubies seen (bit mask),
 * t4 twist sum modulo 3, a3 orientation rank (first six twists, base 3).
 */
    la   t0, input
    la   t1, digits
    li   t2, 0
    li   t3, 0
    li   t4, 0
    li   a3, 0
parse:
    lbu  t5, 0(t0)              # cubie digit, '1'..'7'
    addi t5, t5, -49
    sltiu t6, t5, 7
    beqz t6, invalid
    li   a0, 1
    sll  a0, a0, t5
    and  a1, t3, a0
    bnez a1, invalid            # cubie repeated
    or   t3, t3, a0
    sb   t5, 0(t1)
    lbu  t6, 7(t0)              # twist digit, '1'..'3'
    addi t6, t6, -49
    sltiu a1, t6, 3
    beqz a1, invalid
    add  t4, t4, t6             # sum <= 2 + 2, one subtract reduces it
    addi a1, t4, -3
    bltz a1, parse_sum_ok
    mv   t4, a1
parse_sum_ok:
    li   a1, 6
    beq  t2, a1, parse_next     # the seventh twist is implied, not ranked
    slli a2, a3, 1              # a3 = a3 * 3 + twist
    add  a3, a2, a3
    add  a3, a3, t6
parse_next:
    addi t0, t0, 1
    addi t1, t1, 1
    addi t2, t2, 1
    li   a1, 7
    bne  t2, a1, parse
    lbu  t5, 7(t0)              # input[14] must end the string
    bnez t5, invalid
    bnez t4, invalid            # twist sum must be 0 modulo 3

#ifdef LOOP_RANK
/* ---- Permutation rank, as rank_state() in solver.c ---------------------
 * r = r * (7 - i) + (number of later digits smaller than digit i).
 * The multiplier changes with i, so it is repeated addition: this runs
 * once per query, never in the search.
 */
    la   t0, digits
    addi t6, t0, 7              # end of digits
    li   a2, 0                  # r
    li   t2, 7                  # multiplier 7 - i
rank_digit:
    lbu  t5, 0(t0)
    addi t1, t0, 1
    li   a5, 0                  # smaller-digit count
rank_count:
    bgeu t1, t6, rank_counted
    lbu  a1, 0(t1)
    sltu a1, a1, t5
    add  a5, a5, a1
    addi t1, t1, 1
    j    rank_count
rank_counted:
    li   a4, 0                  # a4 = r * multiplier
    mv   a1, t2
rank_scale:
    add  a4, a4, a2
    addi a1, a1, -1
    bnez a1, rank_scale
    add  a2, a4, a5
    addi t0, t0, 1
    addi t2, t2, -1
    bnez t2, rank_digit
#else
/* ---- Permutation rank (asm v3) ----------------------------------------
 * Part A: counts[i] = how many digits after digits[i] are smaller, for
 * i = 0..5. The last digit has nothing after it, so its count is always 0
 * and is not stored.
 * Part B: r = counts[0], then r = r * k + counts[i] for k = 6, 5, 4, 3, 2,
 * each constant multiply written as shifts and adds: no loop, no multiply.
 * Runs once per query, never in the search.
 */
    la   t0, digits             # t0 = &digits[i], i = 0
    addi t2, t0, 6              # outer stop: digits + 6, so i = 0..5
    addi t6, t0, 7              # inner stop: digits + 7, so j reaches 6
    la   t3, counts             # t3 = &counts[i]
count_digit:
    lbu  t5, 0(t0)              # cur = digits[i]
    addi t1, t0, 1              # t1 = &digits[j], j = i + 1
    li   a5, 0                  # count = 0
count_later:
    bgeu t1, t6, count_stored   # j = 7: nothing left to compare
    lbu  a1, 0(t1)              # a1 = digits[j]
    sltu a1, a1, t5             # a1 = 1 if digits[j] < cur, else 0
    add  a5, a5, a1             # count += a1, no branch needed
    addi t1, t1, 1              # j++
    j    count_later
count_stored:
    sb   a5, 0(t3)              # counts[i] = count
    addi t0, t0, 1              # i++ on the digits side
    addi t3, t3, 1              # i++ on the counts side
    bltu t0, t2, count_digit    # i < 6: next digit

/* Part B: r = counts[0], then r = r * k + counts[i] for k = 6, 5, 4, 3, 2.
 * Each multiply by a constant is shifts and adds: slli by k multiplies by
 * 2 to the power k. */
    la   t3, counts             # t3 = &counts[0], A left it at counts + 6
    lbu  a2, 0(t3)              # r = counts[0]
    slli t0, a2, 1              # t0 = 2r
    add  t0, t0, a2             # t0 = 3r
    slli a2, t0, 1              # r = 6r, as 3r * 2
    lbu  a5, 1(t3)
    add  a2, a2, a5             # r = 6r + counts[1]
    slli t0, a2, 2              # t0 = 4r
    add  a2, t0, a2             # r = 5r, as 4r + r
    lbu  a5, 2(t3)
    add  a2, a2, a5             # r = 5r + counts[2]
    slli a2, a2, 2              # r = 4r
    lbu  a5, 3(t3)
    add  a2, a2, a5             # r = 4r + counts[3]
    slli t0, a2, 1              # t0 = 2r
    add  a2, a2, t0             # r = 3r, as r + 2r
    lbu  a5, 4(t3)
    add  a2, a2, a5             # r = 3r + counts[4]
    slli a2, a2, 1              # r = 2r
    lbu  a5, 5(t3)
    add  a2, a2, a5             # r = 2r + counts[5], the permutation rank
#endif
#ifdef INDEX_TABLES
    mv   gp, a2                 # root permutation rank
    mv   tp, a3                 # root orientation rank
#else
    slli gp, a2, 1              # root permutation rank, scaled by 2
    slli tp, a3, 1              # root orientation rank, scaled by 2
#endif

/* ---- Tables and the root bound ----------------------------------------- */
    la   s0, perm_turn
    la   s1, ori_turn
    la   s2, perm_pdb
    la   s3, ori_pdb
    la   s4, perimeter_offset
    la   s5, perimeter_entry
    la   s8, frames
    li   s11, PERM_FACE_BYTES
    li   a0, 0                  # path length if the root is solved
    add  t0, s2, gp
    LOAD_PDB t0, 0(t0)
    add  t1, s3, tp
    LOAD_PDB t1, 0(t1)
    bgeu t0, t1, root_h
    mv   t0, t1
root_h:
    beqz t0, report             # solved already
#ifndef HEURISTIC_A0
    li   t1, PERIMETER_RADIUS
    bltu t1, t0, root_bound     # h0 above the radius: not in the perimeter
    mv   a2, gp
    mv   a3, tp
    jal  ra, perimeter_lookup   # t0 = exact distance, or radius + 1
#endif
root_bound:
    mv   s6, t0

/* ---- One bounded depth-first pass per bound ---------------------------- */
pass:
    li   t0, 12
    bgeu s6, t0, no_solution    # nothing within 11 moves: invalid input
    mv   s7, s8
    addi a6, s6, -1
    mv   a4, gp
    mv   a5, tp
    mv   a2, gp
    mv   a3, tp
    li   a0, 0
    li   a1, 0
    li   a7, 3
    mv   s9, s0
    mv   s10, s1

/* Generate the next child: one more quarter turn of face a0 applied to the
 * previous child (or to the node, after a face change).
 */
child:
#ifdef INDEX_TABLES
    slli t0, a2, 1
    add  t0, s9, t0
    lhu  a2, 0(t0)              # perm rank after the turn
    slli t1, a3, 1
    add  t1, s10, t1
    lhu  a3, 0(t1)              # ori rank after the turn
#else
    add  t0, s9, a2
    lhu  a2, 0(t0)              # perm rank after the turn (scaled)
    add  t1, s10, a3
    lhu  a3, 0(t1)              # ori rank after the turn (scaled)
#endif
    add  t0, s2, a2
    LOAD_PDB t0, 0(t0)
    add  t1, s3, a3
    LOAD_PDB t1, 0(t1)
    bgeu t0, t1, child_h
    mv   t0, t1                 # t0 = h0 = max of the two lookups
child_h:
    beqz t0, found              # h0 == 0 only at solved
    bltu a6, t0, next           # h0 > moves left: prune
#ifndef HEURISTIC_A0
#ifdef EAGER_LOOKUP
    /* As the C A3B: look up whenever h0 is within the radius. */
    li   t1, PERIMETER_RADIUS
    bltu t1, t0, descend        # h0 above the radius: not in the perimeter
#else
    /* The perimeter can only raise h to at most radius + 1. With that many
     * moves left it cannot prune, so look up only when fewer are left. */
    li   t1, PERIMETER_OUTSIDE
    bgeu a6, t1, descend
#endif
#if defined(EAGER_LOOKUP) || defined(CALL_LOOKUP)
    jal  ra, perimeter_lookup
    bltu a6, t0, next           # exact distance, or radius + 1: prune
#else
    /* asm v2: the bucket scan inline. Here at most PERIMETER_RADIUS moves
     * are left, so an absent state (h = radius + 1) always prunes, and a
     * larger ori gives entry - (o << 3) >= 8, which prunes the same way:
     * one comparison covers hit, miss, and overshoot. */
#ifdef INDEX_TABLES
    slli t2, a2, 1
    add  t2, s4, t2
    lhu  t3, 0(t2)
    lhu  t4, 2(t2)
    slli t3, t3, 1
    add  t3, s5, t3             # first entry of the bucket
    slli t4, t4, 1
    add  t4, s5, t4             # one past the last
#else
    add  t2, s4, a2
    lhu  t3, 0(t2)              # byte offsets into perimeter_entry
    lhu  t4, 2(t2)
    add  t3, s5, t3             # first entry of the bucket
    add  t4, s5, t4             # one past the last
#endif
    slli t5, a3, ORI_KEY_SHIFT
child_scan:
    bgeu t3, t4, next           # bucket exhausted: absent, prune
    lhu  t6, 0(t3)
    addi t3, t3, 2
    bltu t6, t5, child_scan     # ori below o: keep scanning
    sub  t0, t6, t5             # distance on a match, >= 8 past it
    bltu a6, t0, next           # more than the moves left: prune
#endif
#endif

/* Save this depth, then make the child the node of the next depth. */
descend:
    sw   s9, 0(s7)
    sw   s10, 4(s7)
    sh   a4, 8(s7)
    sh   a5, 10(s7)
    sh   a2, 12(s7)
    sh   a3, 14(s7)
    sb   a0, 16(s7)
    sb   a1, 17(s7)
    addi s7, s7, FRAME
    addi a6, a6, -1
    mv   a7, a0                 # the face just used
    mv   a4, a2
    mv   a5, a3
    li   a0, 0
    li   a1, 0
    mv   s9, s0
    mv   s10, s1
    bnez a7, child              # face 0 allowed unless it was just used
    li   a0, 1
    add  s9, s0, s11
    addi s10, s1, ORI_FACE_BYTES
    j    child

/* Next sibling: the next quarter turn of this face, else the next face
 * (never the one just used), else backtrack.
 */
next:
    addi a1, a1, 1
    li   t0, 3
    bne  a1, t0, child
    li   a1, 0
next_face:
    addi a0, a0, 1
    add  s9, s9, s11
    addi s10, s10, ORI_FACE_BYTES
    li   t0, 3
    beq  a0, t0, backtrack      # test before the skip: the root sentinel is 3
    beq  a0, a7, next_face
    mv   a2, a4
    mv   a3, a5
    j    child

backtrack:
    beq  s7, s8, pass_failed
    addi s7, s7, -FRAME
    addi a6, a6, 1
    lw   s9, 0(s7)
    lw   s10, 4(s7)
    lhu  a4, 8(s7)
    lhu  a5, 10(s7)
    lhu  a2, 12(s7)
    lhu  a3, 14(s7)
    lbu  a0, 16(s7)
    lbu  a1, 17(s7)
    lbu  a7, -16(s7)            # face of the depth above (sentinel: 3)
    j    next

pass_failed:
    addi s6, s6, 1
    j    pass

/* ---- Solution: read the path back from the frames ---------------------- */
found:
    la   t2, moves
    mv   t3, s8
copy_move:
    beq  t3, s7, copy_last
    lbu  t0, 16(t3)             # move = face * 3 + turns
    slli t1, t0, 1
    add  t0, t1, t0
    lbu  t1, 17(t3)
    add  t0, t0, t1
    sb   t0, 0(t2)
    addi t2, t2, 1
    addi t3, t3, FRAME
    j    copy_move
copy_last:
    slli t1, a0, 1
    add  t0, t1, a0
    add  t0, t0, a1
    sb   t0, 0(t2)
    la   t0, moves
    sub  a0, t2, t0
    addi a0, a0, 1              # path length

/* ---- Print the moves, check them, report ------------------------------- */
report:
    mv   s6, a0                 # s6 = length from here on
    la   s7, moves
    add  s8, s7, s6             # end of moves
    li   s9, 0                  # moves printed
#if RENDER
    jal  ra, render_init        # show the assembled input before replay
#endif
print_move:
    beq  s7, s8, print_done
    beqz s9, print_name
    li   a0, 32                 # space
    li   a7, 11
    ecall
print_name:
    lbu  t0, 0(s7)
    slli t0, t0, 2
    la   a0, names
    add  a0, a0, t0
    li   a7, 4
    ecall
    addi s7, s7, 1
    addi s9, s9, 1
    j    print_move
print_done:
    la   a0, msg_newline
    li   a7, 4
    ecall

/* Self-check: replay the path through the transition tables from the root
 * and require the solved state (and, if given, the expected length). */
    mv   a2, gp
    mv   a3, tp
    la   s7, moves
replay:
    beq  s7, s8, replay_done
    lbu  t0, 0(s7)
    mv   t1, s0                 # perm row of the face, t0 = turns - 1
    mv   t2, s1
replay_face:
    li   t3, 3
    bltu t0, t3, replay_turn
    addi t0, t0, -3
    add  t1, t1, s11
    addi t2, t2, ORI_FACE_BYTES
    j    replay_face
replay_turn:
#ifdef INDEX_TABLES
    slli t3, a2, 1
    add  t3, t1, t3
    lhu  a2, 0(t3)
    slli t3, a3, 1
    add  t3, t2, t3
    lhu  a3, 0(t3)
#else
    add  t3, t1, a2
    lhu  a2, 0(t3)
    add  t3, t2, a3
    lhu  a3, 0(t3)
#endif
    addi t0, t0, -1
    bgez t0, replay_turn
#if RENDER
    lbu  a0, 0(s7)              # exactly the move the solver returned
    jal  ra, render_step        # one redraw per R/R2/R', B/B2/B', D/D2/D'
#endif
    addi s7, s7, 1
    j    replay
replay_done:
#if RENDER
/* Independently require the displayed cubies to be solved too. */
    la   t0, render_perm
    la   t1, render_ori
    li   t2, 0
render_check_solved:
    lbu  t3, 0(t0)
    bne  t3, t2, report_fail
    lbu  t3, 0(t1)
    bnez t3, report_fail
    addi t0, t0, 1
    addi t1, t1, 1
    addi t2, t2, 1
    li   t3, 8
    bne  t2, t3, render_check_solved
#endif
    or   t0, a2, a3
    bnez t0, report_fail
#ifdef EXPECT
    li   t0, EXPECT
    bne  s6, t0, report_fail
#endif
    la   a0, msg_ok
    j    report_length
report_fail:
    la   a0, msg_fail
report_length:
    li   a7, 4
    ecall
    mv   a0, s6
    li   a7, 1
    ecall
    la   a0, msg_newline
    li   a7, 4
    ecall
    j    exit

invalid:
no_solution:
    la   a0, msg_invalid
    li   a7, 4
    ecall
exit:
    li   a7, 10
    ecall

#ifndef HEURISTIC_A0
/* ---- perimeter_lookup ---------------------------------------------------
 * In:  a2 perm rank, a3 ori rank.  Out: t0 = exact distance if the state
 * is in the perimeter, else PERIMETER_OUTSIDE (radius + 1).  Clobbers
 * t2 to t6.
 * The bucket of p is perimeter_entry[offset[p] .. offset[p + 1]), entries
 * [ori:10][distance:3] ascending, so entry >= (o << 3) means ori >= o.
 */
perimeter_lookup:
#ifdef INDEX_TABLES
    slli t2, a2, 1
    add  t2, s4, t2
    lhu  t3, 0(t2)
    lhu  t4, 2(t2)
    slli t3, t3, 1
    add  t3, s5, t3             # first entry of the bucket
    slli t4, t4, 1
    add  t4, s5, t4             # one past the last
#else
    add  t2, s4, a2
    lhu  t3, 0(t2)              # byte offsets into perimeter_entry
    lhu  t4, 2(t2)
    add  t3, s5, t3             # first entry of the bucket
    add  t4, s5, t4             # one past the last
#endif
    slli t5, a3, ORI_KEY_SHIFT
lookup_scan:
    bgeu t3, t4, lookup_miss
    lhu  t6, 0(t3)
    addi t3, t3, 2
    bltu t6, t5, lookup_scan    # ori below o: keep scanning
    sub  t0, t6, t5             # equal ori leaves just the distance
    li   t6, 8
    bltu t0, t6, lookup_done
lookup_miss:
    li   t0, PERIMETER_OUTSIDE
lookup_done:
    ret
#endif

#if RENDER
/* Renderer uses a0,a1,a4-a7,t0-t6 only; a2/a3 (replay ranks), all s
 * registers, gp/tp stay intact. No recursion, heap, multiply or division. */
render_init:
    addi sp, sp, -16
    sw   ra, 12(sp)
    la   t0, input
    la   t1, render_perm
    la   t2, render_ori
    sb   zero, 0(t1)            # the omitted corner 0 stays fixed
    sb   zero, 0(t2)
    li   t3, 1
render_init_corner:
    lbu  t4, 0(t0)
    addi t4, t4, -48            # global cubie numbers 1..7
    add  t5, t1, t3
    sb   t4, 0(t5)
    lbu  t4, 7(t0)
    addi t4, t4, -49            # twists 0..2
    add  t5, t2, t3
    sb   t4, 0(t5)
    addi t0, t0, 1
    addi t3, t3, 1
    li   t4, 8
    bne  t3, t4, render_init_corner
    li   t0, LED_MATRIX_0_BASE
    li   t1, LED_MATRIX_0_HEIGHT
render_clear_row:
    li   t2, LED_MATRIX_0_WIDTH
render_clear_pixel:
    sw   zero, 0(t0)
    addi t0, t0, 4
    addi t2, t2, -1
    bnez t2, render_clear_pixel
    addi t1, t1, -1
    bnez t1, render_clear_row
    jal  ra, render_draw
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

/* a0 = move ID 0..8. Buffer all seven destinations before replacing the
 * current state, so a cycle never reads a corner already overwritten. */
render_step:
    addi sp, sp, -16
    sw   ra, 12(sp)
    la   a4, render_source
    la   a5, render_twist
    li   a6, 3
render_move_face:
    bltu a0, a6, render_move_turns
    addi a0, a0, -3
    addi a4, a4, 7
    addi a5, a5, 7
    j    render_move_face
render_move_turns:
    addi a7, a0, 1              # one, two or three quarter turns
render_quarter:
    li   a0, 0
render_corner:
    add  t0, a4, a0
    lbu  t0, 0(t0)              # source corner position
    la   t1, render_perm
    add  t1, t1, t0
    lbu  t1, 0(t1)
    la   t2, render_next_perm
    add  t2, t2, a0
    sb   t1, 1(t2)
    la   t1, render_ori
    add  t1, t1, t0
    lbu  t1, 0(t1)
    add  t2, a5, a0
    lbu  t2, 0(t2)
    add  t1, t1, t2
    addi t2, t1, -3
    bltz t2, render_twist_ok
    mv   t1, t2
render_twist_ok:
    la   t2, render_next_ori
    add  t2, t2, a0
    sb   t1, 1(t2)
    addi a0, a0, 1
    li   t0, 7
    bne  a0, t0, render_corner
    li   a0, 1
    la   t0, render_next_perm
    la   t1, render_next_ori
    la   t2, render_perm
    la   t3, render_ori
render_copy_corner:
    add  t4, t0, a0
    lbu  t4, 0(t4)
    add  t5, t2, a0
    sb   t4, 0(t5)
    add  t4, t1, a0
    lbu  t4, 0(t4)
    add  t5, t3, a0
    sb   t4, 0(t5)
    addi a0, a0, 1
    li   t4, 8
    bne  a0, t4, render_copy_corner
    addi a7, a7, -1
    bnez a7, render_quarter
    jal  ra, render_draw
    lw   ra, 12(sp)
    addi sp, sp, 16
    ret

/* Paint all 24 stickers. address = BASE + 4*(y*WIDTH+x), row-major;
 * y*WIDTH is repeated addition here (at most 19 iterations per sticker).
 * Within the 4x3 rectangle each row begins one WIDTH-word stride later. */
render_draw:
    la   a4, render_facelets
    addi a5, a4, 96
render_sticker:
    lbu  t0, 0(a4)              # position
    la   t1, render_perm
    add  t1, t1, t0
    lbu  t1, 0(t1)              # cubie
    la   t2, render_ori
    add  t2, t2, t0
    lbu  t2, 0(t2)
    lbu  t3, 1(a4)              # destination basis slot
    add  t2, t2, t3
    addi t3, t2, -3
    bltz t3, render_slot_ok
    mv   t2, t3
render_slot_ok:
    slli t3, t1, 1
    add  t1, t1, t3             # cubie * 3
    add  t1, t1, t2
    la   t3, render_basis
    add  t1, t1, t3
    lbu  t1, 0(t1)              # color ID
    slli t1, t1, 2
    la   t3, render_palette
    add  t1, t1, t3
    lw   a6, 0(t1)              # RGB
    lbu  t0, 2(a4)              # x
    lbu  t1, 3(a4)              # y
    li   t2, LED_MATRIX_0_WIDTH
render_row_offset:
    beqz t1, render_address
    add  t0, t0, t2
    addi t1, t1, -1
    j    render_row_offset
render_address:
    slli t0, t0, 2
    li   t1, LED_MATRIX_0_BASE
    add  t0, t0, t1
    slli t2, t2, 2              # row stride in bytes
    li   t3, 3
render_rect_row:
    mv   t4, t0
    li   t5, 4
render_rect_pixel:
    sw   a6, 0(t4)
    addi t4, t4, 4
    addi t5, t5, -1
    bnez t5, render_rect_pixel
    add  t0, t0, t2
    addi t3, t3, -1
    bnez t3, render_rect_row
    addi a4, a4, 4
    bne  a4, a5, render_sticker
/* Delay is GUI-only. Set RENDER_DELAY=0 for scripted renderer checks. */
    li   t0, RENDER_DELAY
    beqz t0, render_draw_done
render_pause:
    addi t0, t0, -1
    bnez t0, render_pause
render_draw_done:
    ret
#endif
