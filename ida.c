/* Host driver for the A0 search (ida.h) and its correctness gates.
 *
 *   ./ida STATE      print a shortest solution, like ./solver
 *   ./ida --gates    H1, H2, and full-domain H3, plus search-cost statistics
 *   ./ida --stream   print STATE|MOVES for every state (pipe to
 *                    ./verify --solutions for an independent H3 check)
 *   ./ida --hardest  print every distance-11 state with its search cost,
 *                    most expensive first (input list for target sweeps)
 *   ./ida --perimeter [--gates | --stream]
 *                    A3 (ida_perimeter.h with perimeter.h): audit the packed
 *                    entries (H2/H4), H1, and optimal paths on the perimeter,
 *                    the reference, and every distance-11 state; --gates
 *                    checks the whole domain; --stream feeds ./verify.
 *                    Variant by binary: ida (A3T), ida_a3d, ida_a3b
 *
 * The distance oracle here is a breadth-first search over the full state
 * graph, built from the factored transitions, independent of the pattern
 * databases it is used to check.
 */
#define main baseline_cli_main
#include "solver.c"
#undef main

#include <inttypes.h>
#include <time.h>

#include "tables.h"
#include "perimeter.h"
#include "ida.h"
#include "ida_perimeter.h"

static void fail(const char *message, uint32_t rank)
{
    fprintf(stderr, "FAIL: %s (rank %" PRIu32 ")\n", message, rank);
    exit(EXIT_FAILURE);
}

static void state_string(uint32_t rank, char out[15])
{
    state_t state;
    unrank_state(rank, &state);
    for (uint8_t i = 0; i < CUBIES; ++i) {
        out[i] = (char) ('1' + state.p[i]);
        out[i + CUBIES] = (char) ('1' + state.o[i]);
    }
    out[14] = '\0';
}

/* Exact distance of every state, by BFS over the full graph. */
static uint8_t *make_distance(void)
{
    uint8_t *distance = malloc(STATES);
    uint32_t *queue = malloc((size_t) STATES * sizeof *queue);
    uint32_t head = 0, tail = 1;
    if (!distance || !queue)
        fail("oracle allocation", 0);
    memset(distance, UINT8_MAX, STATES);
    distance[0] = 0;
    queue[0] = 0;
    while (head < tail) {
        uint32_t here = queue[head++];
        uint16_t p0 = (uint16_t) (here / ORIENTATIONS);
        uint16_t o0 = (uint16_t) (here % ORIENTATIONS);
        for (uint8_t face = 0; face < 3; ++face) {
            uint16_t p = p0, o = o0;
            for (uint8_t power = 0; power < 3; ++power) {
                p = perm_turn[face][p];
                o = ori_turn[face][o];
                uint32_t there = (uint32_t) p * ORIENTATIONS + o;
                if (distance[there] == UINT8_MAX) {
                    distance[there] = (uint8_t) (distance[here] + 1);
                    queue[tail++] = there;
                }
            }
        }
    }
    free(queue);
    if (tail != STATES)
        fail("oracle BFS incomplete", tail);
    return distance;
}

/* H2: every entry filled, solved entry 0, report the maximum. */
static void gate_h2(void)
{
    uint8_t perm_max = 0, ori_max = 0;
    for (uint16_t i = 0; i < PERM_RANKS; ++i) {
        if (perm_pdb[i] == UINT8_MAX)
            fail("perm_pdb entry unfilled", i);
        if (perm_pdb[i] > perm_max)
            perm_max = perm_pdb[i];
        for (uint8_t f = 0; f < 3; ++f)
            if (perm_turn[f][i] >= PERM_RANKS)
                fail("perm_turn entry out of range", i);
    }
    for (uint16_t i = 0; i < ORI_RANKS; ++i) {
        if (ori_pdb[i] == UINT8_MAX)
            fail("ori_pdb entry unfilled", i);
        if (ori_pdb[i] > ori_max)
            ori_max = ori_pdb[i];
        for (uint8_t f = 0; f < 3; ++f)
            if (ori_turn[f][i] >= ORI_RANKS)
                fail("ori_turn entry out of range", i);
    }
    if (perm_pdb[0] != 0 || ori_pdb[0] != 0)
        fail("solved entry is not 0", 0);
    printf("H2 PASS: perm_pdb %d entries (max %u), ori_pdb %d entries "
           "(max %u), solved entries 0, transitions in range\n",
           PERM_RANKS, perm_max, ORI_RANKS, ori_max);
}

/* H1: h(s) <= d(s) for every state. */
static void gate_h1(const uint8_t *distance)
{
    uint64_t h_sum = 0, d_sum = 0;
    uint32_t exact = 0;
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        uint8_t h = ida_h((uint16_t) (rank / ORIENTATIONS),
                          (uint16_t) (rank % ORIENTATIONS));
        if (h > distance[rank])
            fail("heuristic overestimates", rank);
        h_sum += h;
        d_sum += distance[rank];
        exact += h == distance[rank];
    }
    printf("H1 PASS: h <= d for all %d states; mean h %.3f, mean d %.3f, "
           "h == d for %" PRIu32 " states\n",
           STATES, (double) h_sum / STATES, (double) d_sum / STATES, exact);
}

/* Apply moves with the upstream state code, independent of the tables. */
static int path_solves(uint32_t rank, const uint8_t *moves, uint8_t length)
{
    state_t state;
    unrank_state(rank, &state);
    for (uint8_t i = 0; i < length; ++i)
        state = apply_move(state, moves[i]);
    return rank_state(&state) == 0;
}

static double seconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double) now.tv_sec + now.tv_nsec / 1e9;
}

/* H3: the search returns a path of exact length that reaches solved, for
 * every state. Also collects search cost by distance.
 */
static void gate_h3(const uint8_t *distance)
{
    uint32_t worst_gen[MAX_DEPTH + 1] = {0}, worst_exp[MAX_DEPTH + 1] = {0};
    uint32_t worst_rank[MAX_DEPTH + 1] = {0}, count[MAX_DEPTH + 1] = {0};
    uint64_t sum_gen[MAX_DEPTH + 1] = {0};
    double start = seconds();
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        uint8_t moves[MAX_DEPTH];
        ida_stats_t stats = {0, 0};
        uint8_t length = ida_solve((uint16_t) (rank / ORIENTATIONS),
                                   (uint16_t) (rank % ORIENTATIONS), moves,
                                   &stats);
        if (length != distance[rank])
            fail("search length differs from exact distance", rank);
        if (!path_solves(rank, moves, length))
            fail("search path does not reach solved", rank);
        ++count[length];
        sum_gen[length] += stats.generated;
        if (stats.generated > worst_gen[length]) {
            worst_gen[length] = stats.generated;
            worst_exp[length] = stats.expanded;
            worst_rank[length] = rank;
        }
    }
    double elapsed = seconds() - start;
    printf("H3 PASS: all %d states solved with exact length (%.1f s)\n",
           STATES, elapsed);
    printf("\n d     states   mean generated   worst generated   worst "
           "expanded   worst state\n");
    for (uint8_t d = 0; d <= MAX_DEPTH; ++d) {
        char name[15];
        state_string(worst_rank[d], name);
        printf("%2u %10" PRIu32 " %16.0f %17" PRIu32 " %16" PRIu32 "   %s\n",
               d, count[d], count[d] ? (double) sum_gen[d] / count[d] : 0.0,
               worst_gen[d], worst_exp[d], name);
    }
}

static void stream(void)
{
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        uint8_t moves[MAX_DEPTH];
        ida_stats_t stats = {0, 0};
        char name[15];
        uint8_t length = ida_solve((uint16_t) (rank / ORIENTATIONS),
                                   (uint16_t) (rank % ORIENTATIONS), moves,
                                   &stats);
        state_string(rank, name);
        printf("%s|", name);
        for (uint8_t i = 0; i < length; ++i)
            printf("%s%s", i ? " " : "", move_names[moves[i]]);
        putchar('\n');
    }
}

typedef struct {
    uint32_t rank, generated, expanded;
} cost_t;

static int by_cost(const void *a, const void *b)
{
    const cost_t *x = a, *y = b;
    if (x->generated != y->generated)
        return x->generated < y->generated ? 1 : -1;
    return x->rank < y->rank ? -1 : x->rank > y->rank;
}

static void hardest(const uint8_t *distance)
{
    static cost_t list[STATES];
    uint32_t n = 0;
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        if (distance[rank] != MAX_DEPTH)
            continue;
        uint8_t moves[MAX_DEPTH];
        ida_stats_t stats = {0, 0};
        ida_solve((uint16_t) (rank / ORIENTATIONS),
                  (uint16_t) (rank % ORIENTATIONS), moves, &stats);
        list[n++] = (cost_t) {rank, stats.generated, stats.expanded};
    }
    qsort(list, n, sizeof *list, by_cost);
    puts("# state generated expanded");
    for (uint32_t i = 0; i < n; ++i) {
        char name[15];
        state_string(list[i].rank, name);
        printf("%s %" PRIu32 " %" PRIu32 "\n", name, list[i].generated,
               list[i].expanded);
    }
}

/* A3: the shared core in ida_perimeter.h, run against the generated
 * perimeter.h. The full distance array here only audits the table and
 * checks results; the search never sees it.
 */
#if defined(PERIMETER_TAIL)
#define A3_NAME "A3T"
#elif defined(PERIMETER_BUCKET)
#define A3_NAME "A3B"
#else
#define A3_NAME "A3D"
#endif

/* Audit both layouts in perimeter.h against the oracle, whichever variant
 * this binary searches with.
 */
static void perimeter_audit(const uint8_t *distance)
{
    uint32_t expected = 0, previous = 0;
    for (uint32_t rank = 0; rank < STATES; ++rank)
        expected += distance[rank] <= PERIMETER_RADIUS;
    if (PERIMETER_SIZE != expected)
        fail("perimeter cardinality", PERIMETER_SIZE);
    /* Sorted layout: ranks ascending, exact distance, first descending move. */
    for (uint32_t i = 0; i < PERIMETER_SIZE; ++i) {
        uint32_t word = perimeter[i], rank = word >> 7;
        uint8_t d = (uint8_t) (word & 7), move = (uint8_t) ((word >> 3) & 15);
        if (rank >= STATES || (i && rank <= previous) || d != distance[rank] ||
            d > PERIMETER_RADIUS)
            fail("sorted entry audit", rank);
        if (!d) {
            if (rank || move != 9)
                fail("solved perimeter entry", rank);
        } else {
            /* Unpacked reference: the first move, in move_names order, that
             * descends, computed with the upstream state code. */
            state_t state;
            unrank_state(rank, &state);
            uint8_t reference;
            for (reference = 0; reference < MOVES; ++reference) {
                state_t next = apply_move(state, reference);
                if (distance[rank_state(&next)] == d - 1)
                    break;
            }
            if (reference == MOVES || move != reference)
                fail("packed move differs from unpacked reference", rank);
        }
        previous = rank;
    }
    /* Bucket layout: offsets cover every entry, each bucket holds exactly
     * the perimeter states of its permutation, ori ascending. */
    if (perimeter_offset[0] != 0 ||
        perimeter_offset[PERMUTATIONS] != PERIMETER_SIZE)
        fail("bucket offsets do not cover the entries", 0);
    for (uint32_t p = 0; p < PERMUTATIONS; ++p) {
        uint32_t members = 0;
        for (uint32_t o = 0; o < ORIENTATIONS; ++o)
            members += distance[p * ORIENTATIONS + o] <= PERIMETER_RADIUS;
        if (perimeter_offset[p + 1] < perimeter_offset[p] ||
            perimeter_offset[p + 1] - perimeter_offset[p] != members)
            fail("bucket size", p);
        for (uint32_t i = perimeter_offset[p]; i < perimeter_offset[p + 1];
             ++i) {
            uint32_t o = perimeter_entry[i] >> 3, d = perimeter_entry[i] & 7;
            if (o >= ORIENTATIONS || d != distance[p * ORIENTATIONS + o] ||
                (i > perimeter_offset[p] && o <= perimeter_entry[i - 1] >> 3))
                fail("bucket entry audit", p * ORIENTATIONS + o);
        }
    }
    printf("%s H2/H4 PASS: %d entries, complete radius %d; sorted layout "
           "(ranks, distances, moves) and bucket layout (offsets, ori, "
           "distances) match the oracle\n",
           A3_NAME, PERIMETER_SIZE, PERIMETER_RADIUS);
}

static void perimeter_h1(const uint8_t *distance)
{
    uint64_t h_sum = 0;
    uint32_t exact = 0;
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        uint32_t entry;
        uint8_t h = perimeter_h((uint16_t) (rank / ORIENTATIONS),
                                (uint16_t) (rank % ORIENTATIONS), &entry);
        if ((entry != PERIMETER_MISS) != (distance[rank] <= PERIMETER_RADIUS) ||
            (entry != PERIMETER_MISS && (entry & 7) != distance[rank]))
            fail("perimeter accessor differs from oracle", rank);
        if (h > distance[rank])
            fail("perimeter heuristic overestimates", rank);
        h_sum += h;
        exact += h == distance[rank];
    }
    printf(A3_NAME " H1 PASS: h <= d for all %d states; mean h %.3f; "
           "h == d for %" PRIu32 " states\n",
           STATES, (double) h_sum / STATES, exact);
}

/* H3 over the whole domain (full) or over the perimeter, the reference
 * vector, and every distance-11 state (quick), with an A0 comparison on the
 * distance-11 states.
 */
static void perimeter_h3(const uint8_t *distance, int full)
{
    uint32_t checked = 0, hard = 0, worst = 0, worst_rank = 0, base_worst = 0;
    uint64_t sum = 0, base_sum = 0;
    double start = seconds();
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        if (!full && distance[rank] > PERIMETER_RADIUS &&
            distance[rank] != MAX_DEPTH && rank != 729U * 720U)
            continue; /* 729 * 720 is the rank of 21345671111111 */
        uint16_t p = (uint16_t) (rank / ORIENTATIONS);
        uint16_t o = (uint16_t) (rank % ORIENTATIONS);
        uint8_t moves[MAX_DEPTH];
        ida_stats_t stats = {0, 0};
        uint8_t length = perimeter_solve(p, o, moves, &stats);
        if (length != distance[rank] || !path_solves(rank, moves, length))
            fail(A3_NAME " path not optimal or not solved", rank);
        ++checked;
        if (distance[rank] != MAX_DEPTH)
            continue;
        ida_stats_t base = {0, 0};
        ida_solve(p, o, moves, &base);
        ++hard;
        sum += stats.generated;
        base_sum += base.generated;
        if (stats.generated > worst) {
            worst = stats.generated;
            worst_rank = rank;
        }
        if (base.generated > base_worst)
            base_worst = base.generated;
    }
    if (hard != 2644 || (full && checked != STATES))
        fail(A3_NAME " coverage incomplete", checked);
    char name[15];
    state_string(worst_rank, name);
    printf(A3_NAME " %s PASS: %" PRIu32 " optimal paths (%.1f s)\n",
           full ? "H3 full-domain" : "path subset", checked,
           seconds() - start);
    printf("D11 generated: A0 mean %.1f worst %" PRIu32 "; " A3_NAME " mean %.1f worst "
           "%" PRIu32 " (%s); host counts, not target instructions\n",
           (double) base_sum / hard, base_worst, (double) sum / hard, worst,
           name);
}

static void perimeter_stream(void)
{
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        uint8_t moves[MAX_DEPTH];
        ida_stats_t stats = {0, 0};
        char name[15];
        uint8_t length = perimeter_solve((uint16_t) (rank / ORIENTATIONS),
                                         (uint16_t) (rank % ORIENTATIONS),
                                         moves, &stats);
        state_string(rank, name);
        printf("%s|", name);
        for (uint8_t i = 0; length != NOT_FOUND && i < length; ++i)
            printf("%s%s", i ? " " : "", move_names[moves[i]]);
        putchar('\n');
    }
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "--perimeter")) {
        int gates = argc == 3 && !strcmp(argv[2], "--gates");
        if (argc == 3 && !strcmp(argv[2], "--stream")) {
            perimeter_stream();
            return output_failed();
        }
        if (argc != 2 && !gates) {
            fprintf(stderr, "usage: %s --perimeter [--gates | --stream]\n",
                    argv[0]);
            return 2;
        }
        uint8_t *distance = make_distance();
        gate_h2();
        perimeter_audit(distance);
        perimeter_h1(distance);
        perimeter_h3(distance, gates);
        free(distance);
        return output_failed();
    }
    if (argc == 2 && !strcmp(argv[1], "--gates")) {
        uint8_t *distance = make_distance();
        gate_h2();
        gate_h1(distance);
        gate_h3(distance);
        free(distance);
        return output_failed();
    }
    if (argc == 2 && !strcmp(argv[1], "--stream")) {
        stream();
        return output_failed();
    }
    if (argc == 2 && !strcmp(argv[1], "--hardest")) {
        uint8_t *distance = make_distance();
        hardest(distance);
        free(distance);
        return output_failed();
    }
    state_t state;
    if (argc != 2 || !parse_state(argv[1], &state)) {
        fprintf(stderr,
                "usage: %s PPPPPPPOOOOOOO | --gates | --stream | --hardest | "
                "--perimeter [--gates | --stream]\n",
                argc > 0 && argv[0] ? argv[0] : "ida");
        return 2;
    }
    uint32_t rank = rank_state(&state);
    uint8_t moves[MAX_DEPTH];
    ida_stats_t stats = {0, 0};
    uint8_t length = ida_solve((uint16_t) (rank / ORIENTATIONS),
                               (uint16_t) (rank % ORIENTATIONS), moves, &stats);
    for (uint8_t i = 0; i < length; ++i)
        printf("%s%s", i ? " " : "", move_names[moves[i]]);
    putchar('\n');
    fprintf(stderr, "expanded %" PRIu32 ", generated %" PRIu32 "\n",
            stats.expanded, stats.generated);
    return output_failed();
}
