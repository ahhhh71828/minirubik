/* A3 search core: A0's IDA* with a perimeter of every state within
 * PERIMETER_RADIUS moves of solved used as an exact heuristic.
 *
 * Shared by the host driver (ida.c) and the RV32I build (rv32/main.c).
 * Include tables.h, perimeter.h, and ida.h first. No heap, no recursion,
 * no division: freestanding code for the target.
 *
 * Three variants, one factor changed at a time:
 *   A3T  -DPERIMETER_TAIL    sorted entries, binary search; a hit within
 *                            the bound appends the stored moves to solved
 *   A3D  (default)           sorted entries, binary search; distances only,
 *                            IDA* searches on to solved
 *   A3B  -DPERIMETER_BUCKET  per-permutation buckets, linear scan;
 *                            distances only, IDA* searches on to solved
 *   -DBYTE_TABLES (with A3B or A3F, target only): the byte-offset layout
 *        of tables_bytes.h, as in the hand-written solver.
 *   A3F  A3B plus -DPERIMETER_LAZY: inside the search, look a child up only
 *        when its A0 bound h0 and the moves left are both within the
 *        radius. The perimeter raises h to at most PERIMETER_RADIUS + 1,
 *        so with more moves left than the radius no lookup could prune and
 *        the child descends either way; the search tree is unchanged.
 *        The root bound and the H1 check still use the full perimeter_h.
 *
 * Heuristic: if A0's lower bound h exceeds the radius, the state cannot be
 * in the perimeter and h stands. Otherwise look it up. Inside, the stored
 * distance is exact; outside, every state within the radius is listed, so
 * the true distance is at least PERIMETER_RADIUS + 1, which here is also
 * max(h, PERIMETER_RADIUS + 1). Neither case overestimates.
 */
#ifndef IDA_PERIMETER_H
#define IDA_PERIMETER_H

#include <stdint.h>

#if defined(PERIMETER_TAIL) && defined(PERIMETER_BUCKET)
#error "the stored tail needs the sorted layout, which holds the moves"
#endif
#if defined(PERIMETER_TAIL) && defined(PERIMETER_LAZY)
#error "the stored tail needs every in-bound perimeter hit"
#endif
#if defined(BYTE_TABLES) && !defined(PERIMETER_BUCKET)
#error "the byte-offset layout exists only for the bucketed perimeter"
#endif

#define PERIMETER_MISS 0xFFFFFFFFu

#ifdef PERIMETER_BUCKET
/* Scan the bucket of permutation rank p, ori ascending. Returns the entry
 * [ori:10][distance:3] or MISS.
 */
#ifdef BYTE_TABLES
/* Byte-offset layout: p and o are scaled by 2, the offsets are byte
 * offsets, and an entry's key ori << 3 equals o << 2.
 */
static uint32_t perimeter_find(uint16_t p, uint16_t o)
{
    const uint8_t *base = (const uint8_t *) perimeter_entry;
    const uint8_t *q = base + AT16(perimeter_offset, p);
    const uint8_t *end = base + AT16(perimeter_offset, p + 2U);
    uint16_t key = (uint16_t) (o << 2);
    for (; q < end; q += 2) {
        uint16_t entry = *(const uint16_t *) q;
        if (entry >= key)
            return entry - key < 8U ? entry : PERIMETER_MISS;
    }
    return PERIMETER_MISS;
}
#else
static uint32_t perimeter_find(uint16_t p, uint16_t o)
{
    for (uint16_t i = perimeter_offset[p]; i < perimeter_offset[p + 1]; ++i) {
        uint16_t entry = perimeter_entry[i];
        if ((entry >> 3) >= o)
            return (entry >> 3) == o ? entry : PERIMETER_MISS;
    }
    return PERIMETER_MISS;
}
#endif
#else
/* Binary search over entries sorted by rank p * 729 + o. Returns the entry
 * [rank:22][move:4][distance:3] or MISS.
 */
static uint32_t perimeter_find(uint16_t p, uint16_t o)
{
    uint32_t key = (uint32_t) p * 729U + o;
    uint32_t lo = 0, hi = PERIMETER_SIZE;
    while (lo < hi) {
        uint32_t mid = (lo + hi) >> 1;
        if ((perimeter[mid] >> 7) < key)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo < PERIMETER_SIZE && (perimeter[lo] >> 7) == key
               ? perimeter[lo]
               : PERIMETER_MISS;
}
#endif

static uint8_t perimeter_h(uint16_t p, uint16_t o, uint32_t *entry)
{
    uint8_t h = ida_h(p, o);
    *entry = PERIMETER_MISS;
    if (h > PERIMETER_RADIUS)
        return h;
    *entry = perimeter_find(p, o);
    return *entry == PERIMETER_MISS ? (uint8_t) (PERIMETER_RADIUS + 1)
                                    : (uint8_t) (*entry & 7U);
}

#ifdef PERIMETER_TAIL
/* Append the stored path from a perimeter state to solved. Each step must
 * land on a perimeter entry one closer; anything else returns NOT_FOUND,
 * which the host gates report as a failure.
 */
static uint8_t perimeter_tail(uint16_t p, uint16_t o, uint32_t entry,
                              uint8_t depth, uint8_t *moves)
{
    while (entry & 7U) {
        uint8_t d = (uint8_t) (entry & 7U);
        uint8_t move = (uint8_t) ((entry >> 3) & 15U);
        if (move >= 9 || depth >= MAX_DEPTH)
            return NOT_FOUND;
        moves[depth++] = move;
        uint8_t face = 0, power = move; /* move = face * 3 + power */
        while (power >= 3) {
            power = (uint8_t) (power - 3);
            ++face;
        }
        for (uint8_t q = 0; q <= power; ++q) {
            p = TURN(perm_turn, face, p);
            o = TURN(ori_turn, face, o);
        }
        entry = perimeter_find(p, o);
        if (entry == PERIMETER_MISS || (entry & 7U) != d - 1U)
            return NOT_FOUND;
    }
    return p == 0 && o == 0 ? depth : NOT_FOUND;
}
#endif

/* A0's pass with perimeter_h in place of ida_h. Same frames, same move
 * order, same same-face rule. Without the stored tail the goal test is
 * A0's: h == 0 only at solved. Inside the perimeter h is exact, so only a
 * child one step closer passes g + h <= bound and the pass walks straight
 * down to solved.
 */
static uint8_t perimeter_pass(uint16_t p0, uint16_t o0, uint8_t bound,
                              uint8_t *moves, ida_stats_t *stats)
{
    uint16_t node_p[MAX_DEPTH + 1], node_o[MAX_DEPTH + 1];
    uint16_t child_p[MAX_DEPTH + 1], child_o[MAX_DEPTH + 1];
    uint8_t face[MAX_DEPTH + 1], turns[MAX_DEPTH + 1];
    unsigned depth = 0;

    node_p[0] = child_p[0] = p0;
    node_o[0] = child_o[0] = o0;
    face[0] = 0;
    turns[0] = 0;
    ++stats->expanded;
    for (;;) {
        if (face[depth] == 3) {
            if (depth == 0)
                return NOT_FOUND;
            --depth;
            goto next_child;
        }
        if (depth > 0 && face[depth] == face[depth - 1]) {
            ++face[depth];
            continue;
        }
        {
            uint8_t f = face[depth];
            uint16_t p = TURN(perm_turn, f, child_p[depth]);
            uint16_t o = TURN(ori_turn, f, child_o[depth]);
            child_p[depth] = p;
            child_o[depth] = o;
            ++stats->generated;
            uint32_t entry;
#ifdef PERIMETER_LAZY
            /* depth + 1 <= bound: a node was entered only with h >= 1 left */
            unsigned left = bound - (depth + 1U);
            uint8_t h = ida_h(p, o);
            if (h <= left && left <= PERIMETER_RADIUS) {
                entry = perimeter_find(p, o);
                h = entry == PERIMETER_MISS ? (uint8_t) (PERIMETER_RADIUS + 1)
                                            : (uint8_t) (entry & 7U);
            }
#else
            uint8_t h = perimeter_h(p, o, &entry);
#endif
            if (depth + 1U + h <= bound) {
                moves[depth] = (uint8_t) (f * 3U + turns[depth]);
#ifdef PERIMETER_TAIL
                if (entry != PERIMETER_MISS)
                    return perimeter_tail(p, o, entry, (uint8_t) (depth + 1U),
                                          moves);
#else
                if (h == 0)
                    return (uint8_t) (depth + 1U);
#endif
                ++depth;
                ++stats->expanded;
                node_p[depth] = child_p[depth] = p;
                node_o[depth] = child_o[depth] = o;
                face[depth] = 0;
                turns[depth] = 0;
                continue;
            }
        }
    next_child:
        if (++turns[depth] == 3) {
            turns[depth] = 0;
            ++face[depth];
            child_p[depth] = node_p[depth];
            child_o[depth] = node_o[depth];
        }
    }
}

static uint8_t perimeter_solve(uint16_t p, uint16_t o, uint8_t *moves,
                               ida_stats_t *stats)
{
    uint32_t entry;
    uint8_t bound = perimeter_h(p, o, &entry);
#ifdef PERIMETER_TAIL
    if (entry != PERIMETER_MISS)
        return perimeter_tail(p, o, entry, 0, moves);
#else
    if (bound == 0)
        return 0;
#endif
    for (; bound <= MAX_DEPTH; ++bound) {
        uint8_t length = perimeter_pass(p, o, bound, moves, stats);
        if (length != NOT_FOUND)
            return length;
    }
    return NOT_FOUND;
}

#endif
