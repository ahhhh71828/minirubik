# Assignment 1: Optimizations and RISC-V Assembly

Contributed by ahhhh71828 · Fork: [ahhhh71828/minirubik](https://github.com/ahhhh71828/minirubik) · Baseline: upstream commit [`231796c`](https://github.com/sysprog21/minirubik/commit/231796c)

| Item | Value |
| :--- | :--- |
| Ripes | continuous build `v2.2.6-106-g5b8a616` (2026-08-18), macOS universal2 |
| RISC-V toolchain | Homebrew `riscv64-elf-gcc` 16.2.0 (prefix differs from `riscv64-unknown-elf-gcc`) |
| Host | Apple M5, 32 GB RAM, macOS |
| Retired instructions | `--iret` on the Ripes build above, same input, renderer compiled out |
| Code size | bytes of linked `.text`, renderer compiled out (defined now, reported from stage 4) |

> Status: this revision covers the state-space model and stage 1. Stages 2–4, the LED matrix, and the pipeline walkthrough follow in later revisions.

## 1. The State Space

### 1.1 Cubies, positions, and the group

A 2×2×2 cube has eight corner cubies and nothing else. The solver fixes one corner (position 0, front-upper-left) and turns only the three faces that do not touch it, R, B, and D. Fixing a corner removes the 24 whole-cube rotations, which change how the cube is held but not how it is scrambled.

The remaining seven cubies are described by two arrays ([`solver.c`](https://github.com/ahhhh71828/minirubik/blob/main/solver.c#L14)):

```c
typedef struct {
    uint8_t p[CUBIES], o[CUBIES];
} state_t;
```

`p[i]` names the cubie at position `i`; `o[i]` is that cubie's twist, 0, 1, or 2 steps of 120°. The reachable states form the group $G = \langle R, B, D \rangle$ acting on these arrays, and its order is

$$|G| = 7! \times 3^6 = 5040 \times 729 = 3{,}674{,}160.$$

The factor $7!$ says every permutation of the seven movable cubies is reachable. The factor is $3^6$ rather than $3^7$ because the seventh twist is determined by the other six.

### 1.2 The orientation invariant is modulo 3, not parity

A quarter turn moves four cubies and adds a fixed twist to each destination ([`solver.c`](https://github.com/ahhhh71828/minirubik/blob/main/solver.c#L59)):

```c
result.o[i] = (uint8_t) ((state.o[from] + twist[face][i]) % 3U);
```

The twist rows sum to $1+2+0+2+1+0+0 = 6$ for R, $0+0+0+1+2+1+2 = 6$ for B, and $0$ for D. Every row sum is $\equiv 0 \pmod 3$, and the move only permutes the existing twists before adding the row, so $\sum_i o_i \bmod 3$ is invariant. The solved state has sum 0, so every reachable state does too.

This excludes two thirds of the $7! \times 3^7 = 11{,}022{,}480$ arrangements that disassembling and reassembling the cubies could produce: every arrangement whose twist sum is 1 or 2 modulo 3, for instance a solved cube with a single corner twisted in place. It is not a parity condition. A quarter turn permutes four positions in a 4-cycle, which is an odd permutation, so both permutation parities are reachable. The sample state `21345671111111` is a single transposition of two corners, and it sits at distance 11.

### 1.3 The Cayley graph and the diameter

Take one vertex per state and one edge per move. In the half-turn metric the generators are the nine moves `R R2 R' B B2 B' D D2 D'`, each of cost 1, and the set is closed under inverses. This is the Cayley graph of $G$ with respect to those generators. The distance from a state to solved is the length of its shortest solution, and the diameter of the graph is the longest such distance.

The baseline computes both facts with a breadth-first search from solved ([`build_table`](https://github.com/ahhhh71828/minirubik/blob/main/solver.c#L191)). A FIFO queue expands states level by level, so the first visit to a state is along a shortest path. Two checks close the argument:

* Completeness. The search ends with `tail == STATES`: all 3,674,160 ranks were reached, which confirms $|G|$ above.
* Diameter. The level counter stops at 11, meaning a level-11 frontier exists (2,644 states) and expanding it discovers nothing new, so no state is at distance 12. The self-test checks this value.

| Distance | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
| :-- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: |
| States | 1 | 9 | 54 | 321 | 1,847 | 9,992 | 50,136 | 227,536 | 870,072 | 1,887,748 | 623,800 | 2,644 |

The distribution, from `report.md` §4, is sharply peaked: 51.4% of states are at distance 9 and 92.0% between 8 and 10. This matters for any search that replaces the table, because a typical query is a hard one.

## 2. Stage 1: Characterizing the Baseline

### 2.1 What the program computes

The baseline answers every query from one precomputed table. `build_table` first derives factored transition tables: one for the permutation rank (5,040 values) and one for the orientation rank (729 values), each with three quarter-turn columns. The factoring is possible because a move's effect on the permutation depends only on the permutation, and its effect on the twists depends only on the twists. It then runs the BFS above and stores, for every state, the inverse of the move that discovered it, which is one step back towards solved. Solving is a walk: look up the move, apply it, repeat until the rank is 0, at most 11 times.

### 2.2 Where the cost lies

Memory, from the declarations in `build_table`:

| Structure | Size | Bytes | Share of peak |
| :--- | :--- | ---: | ---: |
| `queue` (BFS frontier) | 3,674,160 × 4 B | 14,696,640 | 79.8% |
| `toward_solved` (move per state) | 3,674,160 × 1 B | 3,674,160 | 20.0% |
| Transition tables | (5,040 + 729) × 3 × 2 B | 34,614 | 0.2% |
| Peak | | 18,405,414 | |

Time: the BFS expands each of the 3,674,160 states once and generates 9 successors each, about 33 million random-access table updates. The assignment estimates the whole build at about $10^9$ retired instructions. I have not yet measured the baseline itself on the target.

### 2.3 Measuring the target

The two harnesses are in [`stage1/`](https://github.com/ahhhh71828/minirubik/tree/main/stage1). Ripes' built-in assembler rejects `.if` and `.rodata`, so both are preprocessed with `riscv64-elf-gcc -E -P -x assembler-with-cpp` and then assembled by Ripes (`-t asm`); [`run.sh`](https://github.com/ahhhh71828/minirubik/blob/main/stage1/run.sh) wraps one measurement per call.

Simulation rate. [`rate.S`](https://github.com/ahhhh71828/minirubik/blob/main/stage1/rate.S) runs a four-instruction store loop that always writes the same word, so its guest footprint stays constant; the retired count is exactly $4 \times \text{ITERS} + 5$. Ripes' `--exectime` reports the model's wall-clock time.

| Processor model | Retired | Model time | Rate | Assignment's figure |
| :--- | ---: | ---: | ---: | ---: |
| `RV32_ISS` | 100,000,005 | 2,580 ms | 38.8 M/s | 1.09 M/s |
| `RV32_5S` | 5,000,005 | 9,792 ms | 511 k/s | 20.5 k/s |

Host memory per guest byte. [`mem.S`](https://github.com/ahhhh71828/minirubik/blob/main/stage1/mem.S) writes each word of a region once, with a value whose four bytes are all nonzero. Peak resident set size comes from `/usr/bin/time -l`, and the ratio is the slope between a large region and a small control, which cancels Ripes' own footprint.

| Guest bytes written | Retired | Peak host RSS |
| ---: | ---: | ---: |
| 4,096 (control) | 3,079 | 68,796,416 |
| 16,777,216 | 12,582,919 | 1,026,605,056 |
| Slope (host bytes per guest byte) | | 57.1 |

### 2.4 Why the target changes the answer, and by how much

The slope is $(1{,}026{,}605{,}056 - 68{,}796{,}416) / (16{,}777{,}216 - 4{,}096) \approx 57.1$ host bytes per guest byte. Projected onto the baseline's 18,405,414-byte peak, that is about 1.05 GB of host memory for the simulated program, on top of Ripes' own 69 MB. At 38.8 M/s, $10^9$ instructions take about 26 seconds on `RV32_ISS`.

My numbers are 25–36 times faster than the assignment's table, so on this host the baseline is slow and memory-hungry but not infeasible to simulate. The constraints that actually rule it out are the graded ones:

* Static data. The 3,674,160-byte move table alone is 28 times the 128 KiB budget. Packing a distance per state into a nibble still needs 1,794 KiB, 14 times the budget. No packing of a full per-state table fits.
* Instructions. Rebuilding the table on every query costs about $10^9$ instructions, 20 times the $5 \times 10^7$ limit per distance-11 query.
* ISA. The C baseline uses `/` and `%` throughout ranking and unranking, and RV32I has neither.

This is also why `report.md` §7's case for keeping the complete table, written for a host where 3.5 MiB is negligible, does not carry over. The quantity it optimizes, lookup time after a one-off build, is not the quantity graded here. The 34,614 bytes of factored transition tables are the only baseline structure that survives: they fit the budget nearly four times over and leave about 94 KiB for whatever replaces the table.

Two observations from the harnesses carry into later stages:

* Writing fresh guest addresses is slower than rewriting one. The 16 MiB run of `mem.S` retired 12,582,919 instructions in 566 ms of model time, about 22 M/s against 38.8 M/s for `rate.S`, because every store creates new hash entries. A search that keeps its working set small and fixed is cheaper to simulate as well as cheaper in static data.
* The full sweep is affordable. At this host's rate, a query at the $5 \times 10^7$ limit runs in about 1.3 s, so verifying all 2,644 distance-11 states on `RV32_ISS` is a matter of an hour rather than days.

## AI Usage Disclosure

Draft, to be completed at submission (AI Guidelines §4.1). Tools: Claude Code and Codex.

* Explanation of the assignment, the baseline code, and background on heuristic search; a literature survey of candidate techniques. Design choices for stages 2–4 are recorded in later revisions together with which AI suggestions were accepted, rejected, or modified.
* The stage 1 harnesses in `stage1/` were written by Claude. Every number in §2.3 was produced by running them myself.
* The English text of this revision was drafted by Claude from my measurements and our discussion, then reviewed by me.
