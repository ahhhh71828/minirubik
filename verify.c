/* AI-written host verification utility (Codex, 2026-10-05).
 * Build in the repository: make verify; run: ./verify --oracle-only
 * Remove generated binaries with make clean.
 * The complete oracle stays on the host; never link this into the target.
 * Reuse upstream helpers without editing solver.c or its CLI.
 */
#define main baseline_cli_main
#include "solver.c"
#undef main

#include <ctype.h>
#include <inttypes.h>

enum { DIAMETER = 11, INCOMPLETE = 3 };

static void fail(const char *message, uint32_t rank)
{
    fprintf(stderr, "FAIL: %s (rank %" PRIu32 ")\n", message, rank);
    exit(EXIT_FAILURE);
}

static uint8_t *make_oracle(void)
{
    uint8_t diameter;
    uint8_t *toward = build_table(&diameter);
    uint8_t *distance = malloc(STATES);
    uint32_t histogram[DIAMETER + 1] = {0};
    const uint32_t expected[DIAMETER + 1] = {
        1, 9, 54, 321, 1847, 9992, 50136, 227536, 870072,
        1887748, 623800, 2644
    };
    if (!toward || !distance)
        fail("oracle allocation", 0);
    if (diameter != DIAMETER || toward[0] != 0)
        fail("baseline diameter or solved entry", 0);
    memset(distance, UINT8_MAX, STATES);
    distance[0] = 0;

    /* Cache distances along baseline paths. Each chain has at most 11 edges.
     * This derives distances from the existing BFS move table, not a new
     * candidate search. The edge audit below independently checks optimality
     * within the graph defined by the upstream moves.
     */
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        state_t state;
        unrank_state(rank, &state);
        if (!valid(&state) || rank_state(&state) != rank)
            fail("rank/unrank round trip", rank);
        uint32_t chain[DIAMETER], here = rank;
        unsigned length = 0;
        while (distance[here] == UINT8_MAX) {
            if (length == DIAMETER || toward[here] >= MOVES)
                fail("invalid or cyclic baseline path", here);
            chain[length++] = here;
            state = apply_move(state, toward[here]);
            here = rank_state(&state);
            if (here >= STATES)
                fail("successor outside domain", here);
        }
        unsigned depth = distance[here];
        while (length) {
            if (++depth > DIAMETER)
                fail("baseline path exceeds diameter", rank);
            distance[chain[--length]] = (uint8_t) depth;
        }
        ++histogram[distance[rank]];
    }

    /* A solution path supplies an upper bound. On every edge require
     * d(s) <= 1 + d(next); telescoping along any solution supplies a lower
     * bound. Together these establish exact graph distances.
     */
    for (uint32_t rank = 0; rank < STATES; ++rank) {
        state_t state;
        unrank_state(rank, &state);
        for (uint8_t face = 0; face < 3; ++face) {
            state_t next = state;
            for (uint8_t turn = 0; turn < 3; ++turn) {
                next = quarter_turn(next, face);
                uint32_t there = rank_state(&next);
                if (there >= STATES ||
                    distance[rank] > (unsigned) distance[there] + 1U)
                    fail("oracle edge distance inequality", rank);
                if (rank && toward[rank] == face * 3U + turn &&
                    distance[rank] != (unsigned) distance[there] + 1U)
                    fail("baseline path does not decrease distance", rank);
            }
        }
    }
    for (unsigned d = 0; d <= DIAMETER; ++d)
        if (histogram[d] != expected[d])
            fail("distance distribution differs from baseline report", d);

    state_t sample;
    if (!parse_state("21345671111111", &sample) ||
        distance[rank_state(&sample)] != DIAMETER)
        fail("reference vector must have distance 11", 0);
    free(toward);
    printf("ORACLE PASS: %d ranks; %" PRIu64 " edges; diameter %d; "
           "%" PRIu32 " distance-11 states\n",
           STATES, (uint64_t) STATES * MOVES, DIAMETER, histogram[DIAMETER]);
    return distance;
}

static uint32_t input_rank(const char *input, state_t *state)
{
    if (strlen(input) != 14 || !parse_state(input, state))
        fail("invalid 14-character state", 0);
    return rank_state(state);
}

static uint8_t move_id(const char *token)
{
    for (uint8_t move = 0; move < MOVES; ++move)
        if (strcmp(token, move_names[move]) == 0)
            return move;
    fprintf(stderr, "FAIL: unknown move '%s'\n", token);
    exit(EXIT_FAILURE);
}

static uint32_t check_path(const char *input, const uint8_t *moves,
                           unsigned length, const uint8_t *distance)
{
    state_t state;
    uint32_t rank = input_rank(input, &state);
    if (length != distance[rank])
        fail("path length differs from exact distance", rank);
    for (unsigned i = 0; i < length; ++i)
        state = apply_move(state, moves[i]);
    if (rank_state(&state) != 0)
        fail("path does not reach solved", rank);
    return rank;
}

static uint32_t check_records(const uint8_t *distance)
{
    uint8_t *seen = calloc((STATES + 7U) / 8U, 1);
    char line[256];
    uint32_t count = 0, line_number = 0;
    if (!seen)
        fail("coverage allocation", 0);
    while (fgets(line, sizeof line, stdin)) {
        ++line_number;
        if (!strchr(line, '\n') && !feof(stdin))
            fail("record exceeds line buffer", line_number);
        char *input = line;
        while (isspace((unsigned char) *input))
            ++input;
        if (!*input || *input == '#')
            continue;
        char *separator = strchr(input, '|');
        if (!separator)
            fail("expected STATE|MOVE MOVE ...", line_number);
        *separator++ = '\0';
        uint8_t moves[DIAMETER];
        unsigned length = 0;
        for (char *token = strtok(separator, " \t\r\n"); token;
             token = strtok(NULL, " \t\r\n")) {
            if (length == DIAMETER)
                fail("more than 11 moves", line_number);
            moves[length++] = move_id(token);
        }
        uint32_t rank = check_path(input, moves, length, distance);
        uint8_t mask = (uint8_t) (1U << (rank & 7U));
        if (seen[rank >> 3U] & mask)
            fail("duplicate state in solution stream", rank);
        seen[rank >> 3U] |= mask;
        ++count;
    }
    if (ferror(stdin))
        fail("reading solution stream", line_number);
    free(seen);
    printf("PATHS PASS: %" PRIu32 " distinct states; "
           "every path reaches solved with exact length\n", count);
    return count;
}

int main(int argc, char **argv)
{
    int oracle_only = argc == 2 && strcmp(argv[1], "--oracle-only") == 0;
    int records = argc == 2 && strcmp(argv[1], "--solutions") == 0;
    int single = argc >= 3 && strcmp(argv[1], "--check-path") == 0;
    if (argc != 1 && !oracle_only && !records && !single) {
        fprintf(stderr, "usage: %s [--oracle-only | --solutions | "
                "--check-path STATE [MOVE ...]]\n", argv[0]);
        return 2;
    }
    uint8_t moves[DIAMETER];
    unsigned length = 0;
    if (single) {
        state_t state;
        (void) input_rank(argv[2], &state);
        for (int i = 3; i < argc; ++i) {
            if (length == DIAMETER)
                fail("more than 11 moves", 0);
            moves[length++] = move_id(argv[i]);
        }
    }

    uint8_t *distance = make_oracle();
    if (oracle_only) {
        free(distance);
        return output_failed() ? 1 : 0;
    }
    puts("H1 NOT RUN: candidate heuristic not connected");
    puts("H2 NOT RUN: candidate tables not connected");
    if (single) {
        (void) check_path(argv[2], moves, length, distance);
        printf("PATH PASS: %s, %u moves (host only)\n", argv[2], length);
    }
    uint32_t count = records ? check_records(distance) : 0;
    if (records && count == STATES)
        puts("H3 PASS: full-domain supplied paths have exact lengths");
    else
        printf("H3 NOT RUN: full-domain candidate coverage missing "
               "(%" PRIu32 "/%d records)\n", count, STATES);
    puts("H4 NOT RUN: packed accessor not connected");
    puts("T5-T7 NOT RUN: no target execution in this host utility");
    if (!single)
        puts("OVERALL INCOMPLETE: candidate host gates remain unverified");
    free(distance);
    if (output_failed())
        return 1;
    /* Zero is scoped to an explicit oracle-only or single-path check.
     * Default/stream mode cannot silently become a complete gate pass.
     */
    return single ? 0 : INCOMPLETE;
}
