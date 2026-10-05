/* A3 search core: A0's IDA* plus a perimeter of every state within
 * PERIMETER_RADIUS moves of solved.
 *
 * Shared by the host driver (ida.c) and the RV32I build (rv32/main.c).
 * Include tables.h, perimeter.h, and ida.h first. No heap, no recursion,
 * no division: freestanding code for the target.
 *
 * Heuristic: if A0's lower bound h exceeds the radius, the state cannot be
 * in the perimeter and h stands. Otherwise look it up. Inside, the stored
 * distance is exact; outside, every state within the radius is listed, so
 * the true distance is at least PERIMETER_RADIUS + 1. Neither case
 * overestimates.
 *
 * Termination: when a child passes g + h <= bound and is in the perimeter,
 * the stored moves walk it to solved in exactly h steps, so the whole path
 * has length g + h <= bound and the pass can stop there.
 */
#ifndef IDA_PERIMETER_H
#define IDA_PERIMETER_H

#include <stdint.h>

#define PERIMETER_MISS 0xFFFFFFFFu

static inline uint32_t perimeter_key(uint16_t p, uint16_t o)
{
    return (uint32_t) p * 729U + o; /* the upstream state rank */
}

/* Binary search over entries sorted by key; returns the entry or MISS. */
static uint32_t perimeter_find(uint32_t key)
{
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

static uint8_t perimeter_h(uint16_t p, uint16_t o, uint32_t *entry)
{
    uint8_t h = ida_h(p, o);
    *entry = PERIMETER_MISS;
    if (h > PERIMETER_RADIUS)
        return h;
    *entry = perimeter_find(perimeter_key(p, o));
    return *entry == PERIMETER_MISS ? (uint8_t) (PERIMETER_RADIUS + 1)
                                    : (uint8_t) (*entry & 7U);
}

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
            p = perm_turn[face][p];
            o = ori_turn[face][o];
        }
        entry = perimeter_find(perimeter_key(p, o));
        if (entry == PERIMETER_MISS || (entry & 7U) != d - 1U)
            return NOT_FOUND;
    }
    return p == 0 && o == 0 ? depth : NOT_FOUND;
}

/* A0's pass with perimeter_h in place of ida_h and a stored tail as the
 * goal. Same frames, same move order, same same-face rule.
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
            uint16_t p = perm_turn[f][child_p[depth]];
            uint16_t o = ori_turn[f][child_o[depth]];
            child_p[depth] = p;
            child_o[depth] = o;
            ++stats->generated;
            uint32_t entry;
            uint8_t h = perimeter_h(p, o, &entry);
            if (depth + 1U + h <= bound) {
                moves[depth] = (uint8_t) (f * 3U + turns[depth]);
                if (entry != PERIMETER_MISS)
                    return perimeter_tail(p, o, entry, (uint8_t) (depth + 1U),
                                          moves);
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
    if (entry != PERIMETER_MISS)
        return perimeter_tail(p, o, entry, 0, moves);
    for (; bound <= MAX_DEPTH; ++bound) {
        uint8_t length = perimeter_pass(p, o, bound, moves, stats);
        if (length != NOT_FOUND)
            return length;
    }
    return NOT_FOUND;
}

#endif
