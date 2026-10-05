/* RV32I build of the A0 or A3 search for Ripes (GCC reference for
 * stage 4).
 *
 * The input state is fixed at compile time, as the assignment requires:
 *   -DSTATE='"21345671111111"'  [-DEXPECT=11]  [-DPERIMETER for A3]
 * The program parses it, solves it with the same core the host gates
 * checked (ida.h, plus ida_perimeter.h for A3), prints the moves, and
 * validates the result itself: the path must
 * reach solved through the transition tables, and if EXPECT is given its
 * length must match. It prints OK or FAIL.
 *
 * Freestanding: no libc, no multiply or divide (built with -march=rv32i and
 * -nostdlib, so a stray __mulsi3 or __divsi3 fails to link).
 */
#include <stdint.h>

#include "../tables.h"
#include "../ida.h"
#ifdef PERIMETER
#include "../perimeter.h"
#include "../ida_perimeter.h"
#define SOLVE perimeter_solve
#else
#define SOLVE ida_solve
#endif

#ifndef STATE
#define STATE "21345671111111"
#endif

static void ecall_a0(int service, uintptr_t value)
{
    register uintptr_t a0 __asm__("a0") = value;
    register int a7 __asm__("a7") = service;
    __asm__ volatile("ecall" : : "r"(a0), "r"(a7) : "memory");
}

static void print_string(const char *s) { ecall_a0(4, (uintptr_t) s); }
static void print_int(int v) { ecall_a0(1, (uintptr_t) v); }

static const char *const move_names[9] = {"R",  "R2", "R'", "B", "B2",
                                          "B'", "D",  "D2", "D'"};

/* Parse and rank the 14-character input. Returns 0 if it is not a valid
 * state. Ranks match rank_state() in solver.c; the radices are constants,
 * so every multiply is a shift-and-add sequence.
 */
static int parse_rank(const char *s, uint16_t *perm, uint16_t *ori)
{
    uint8_t p[7], seen = 0, sum = 0;
    uint16_t o = 0;
    for (int i = 0; i < 7; ++i) {
        uint8_t c = (uint8_t) (s[i] - '1'), t = (uint8_t) (s[i + 7] - '1');
        if (c > 6 || t > 2 || (seen >> c) & 1U)
            return 0;
        seen |= (uint8_t) (1U << c);
        p[i] = c;
        sum = (uint8_t) (sum + t);
        if (sum >= 3)
            sum = (uint8_t) (sum - 3);
        if (i < 6)
            o = (uint16_t) (o * 3U + t);
    }
    if (s[14] != '\0' || sum != 0)
        return 0;
    uint8_t smaller[6];
    for (int i = 0; i < 6; ++i) {
        uint8_t n = 0;
        for (int j = i + 1; j < 7; ++j)
            n = (uint8_t) (n + (p[j] < p[i]));
        smaller[i] = n;
    }
    *perm = (uint16_t) (((((smaller[0] * 6U + smaller[1]) * 5U + smaller[2]) *
                              4U +
                          smaller[3]) *
                             3U +
                         smaller[4]) *
                            2U +
                        smaller[5]);
    *ori = o;
    return 1;
}

int main(void)
{
    uint16_t p, o;
    if (!parse_rank(STATE, &p, &o)) {
        print_string("FAIL: invalid state\n");
        return 1;
    }
    uint8_t moves[MAX_DEPTH];
    ida_stats_t stats = {0, 0};
    uint8_t length = SOLVE(p, o, moves, &stats);
    if (length == NOT_FOUND) {
        print_string("FAIL: no solution\n");
        return 1;
    }
    for (uint8_t i = 0; i < length; ++i) {
        if (i)
            print_string(" ");
        print_string(move_names[moves[i]]);
    }
    print_string("\n");

    /* Self-check: replay the path one quarter turn at a time. */
    for (uint8_t i = 0; i < length; ++i) {
        uint8_t face = 0, power = moves[i];
        while (power >= 3) {
            power = (uint8_t) (power - 3);
            ++face;
        }
        for (uint8_t q = 0; q <= power; ++q) {
            p = perm_turn[face][p];
            o = ori_turn[face][o];
        }
    }
    int ok = p == 0 && o == 0;
#ifdef EXPECT
    ok = ok && length == EXPECT;
#endif
    print_string(ok ? "OK length " : "FAIL length ");
    print_int(length);
    print_string("\n");
    return !ok;
}
