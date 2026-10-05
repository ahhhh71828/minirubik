# Assignment 1: Optimizations and RISC-V Assembly

Contributed by ahhhh71828 · Fork: [ahhhh71828/minirubik](https://github.com/ahhhh71828/minirubik) · Baseline: upstream commit [`231796c`](https://github.com/sysprog21/minirubik/commit/231796c)

| Item | Value |
| :--- | :--- |
| Ripes | continuous build `v2.2.6-106-g5b8a616` (2026-08-18), macOS universal2 |
| RISC-V toolchain | Homebrew `riscv64-elf-gcc` 16.2.0 (prefix differs from `riscv64-unknown-elf-gcc`) |
| Host | Apple M5, 32 GB RAM, macOS |
| Retired instructions | `--iret` on the Ripes build above, same input, renderer compiled out |
| Code size | bytes of linked `.text`, renderer compiled out (defined now, reported from stage 4) |

> Status: this revision covers the state-space model, stage 1, and stage 2. Stages 3–4, the LED matrix, and the pipeline walkthrough follow in later revisions.

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

## 3. Stage 2: Representation and Search

### 3.1 What stage 1 leaves me with

Stage 1 turns into five requirements for whatever replaces the table:

1. No table over all states, in any packing. The search has to run per query on the target.
2. At most 128 KiB of static data, of which the factored transition tables already take 34,614 bytes.
3. At most $5 \times 10^7$ retired instructions for every distance-11 query, not just a typical one.
4. RV32I only: no multiply, no divide, so no ranking or unranking inside the search loop.
5. No heap and no recursion, so every buffer is sized at assembly time.

And the answer must still be a shortest solution.

### 3.2 Representation: two ranks and four tables

I keep the search state as a pair of ranks, the permutation rank $p \in [0, 5040)$ and the orientation rank $o \in [0, 729)$, exactly the two coordinates the baseline already factors. A move updates each with one table load:

| Table | Shape | Bytes | Answers |
| :--- | :--- | ---: | :--- |
| `perm_turn` | `uint16_t [3][5040]` | 30,240 | the permutation rank after one quarter turn of a face |
| `ori_turn` | `uint16_t [3][729]` | 4,374 | the orientation rank after one quarter turn of a face |
| `perm_pdb` | `uint8_t [5040]` | 5,040 | a lower bound on moves left, from the permutation alone |
| `ori_pdb` | `uint8_t [729]` | 729 | a lower bound on moves left, from the orientation alone |
| Total | | 40,383 | 30.8% of 128 KiB |

[`gen_tables.c`](https://github.com/ahhhh71828/minirubik/blob/main/gen_tables.c) builds all four on the host from the upstream move and rank code and emits them as C arrays.

Three consequences follow from this choice.

* No ranking during search. The state is never expanded back into `p[7]` and `o[7]`. Ranking happens once, when the input string is parsed, and there the radices are constants (720, 120, …, 3), so every multiply becomes shifts and adds.
* No modulo 3 during search. All orientation arithmetic happened on the host when `ori_turn` was built; the search only indexes a table.
* No nine-move tables. `R2` is `R` applied to the `R` child, and `R'` is `R` applied to the `R2` child, so the three children of one face cost one load each from the quarter-turn table. Storing all nine moves instead would take $(5040 + 729) \times 9 \times 2 = 103{,}842$ bytes for no saving in loads.

### 3.3 Search: IDA*

I use iterative-deepening A* (IDA*). Breadth-first search and A* both keep a frontier that, in the worst case, grows with the state space; that frontier is exactly the queue that made up 79.8% of the baseline's memory. IDA* instead runs a depth-first search bounded by $g + h \le \text{bound}$, where $g$ is the number of moves made and $h$ the heuristic, and raises the bound by one after each failed pass. Its memory is one frame per depth, and the depth never exceeds 11.

Without recursion, a pass keeps its path in fixed arrays indexed by depth ([`ida.h`](https://github.com/ahhhh71828/minirubik/blob/main/ida.h)): the node at each depth, the face being tried, the number of quarter turns applied so far, and the latest child. The loop body generates one child and decides:

```c
uint16_t p = perm_turn[f][child_p[depth]];
uint16_t o = ori_turn[f][child_o[depth]];
...
uint8_t h = ida_h(p, o);
if (depth + 1U + h <= bound) {
    moves[depth] = (uint8_t) (f * 3U + turns[depth]);
    if (h == 0)
        return (uint8_t) (depth + 1U);
    ++depth;            /* descend */
```

Two cuts apply before any heuristic:

* Same-face pruning. A face never follows itself, since `R R2` is `R'` and `R R'` is nothing. After the first move this leaves 6 of 9 moves.
* No opposite-face rule. The 3×3×3 trick of ordering commuting opposite faces does not transfer. R, B, and D all share the corner diagonally opposite the fixed one, so no two of them are opposite and none commute.

The goal test is free. $h = 0$ holds exactly when both lookups return 0, which means both coordinates are solved, which is the solved state.

### 3.4 Heuristic: two pattern databases

Each pattern database is a breadth-first search on a projection of the state graph. `perm_pdb` is the distance in the graph whose 5,040 vertices are permutations alone, and `ori_pdb` the distance in the 729-vertex graph of orientations alone. The projections are well defined because, as §2.1 noted, a move's effect on the permutation depends only on the permutation, and likewise for the orientation.

Admissibility. Every move in the full graph maps to a move in the projected graph, so any solution of length $n$ for a state projects onto a path of length $n$ from its projection to the projected goal. The projected distance is therefore at most the true distance, for every state, in both tables. The maximum of two lower bounds is again a lower bound:

$$h(p, o) = \max(\text{perm\_pdb}[p], \text{ori\_pdb}[o]) \le d(p, o).$$

The sum is not, because every move changes both the permutation and the orientation and would be counted twice.

Packing. Both tables hold values at most 7, so a nibble per entry would fit, but it would save only 2,884 bytes of a budget that is two-thirds unused. It would also cost roughly 5 extra instructions on each of the two lookups per child, about $6 \times 10^6$ instructions on the worst query. I keep one byte per entry, so gate H4 does not apply.

| Table | Entries | Max | Mean over all states |
| :--- | ---: | ---: | ---: |
| `perm_pdb` | 5,040 | 7 | 4.862 |
| `ori_pdb` | 729 | 6 | 4.436 |
| $h = \max$ | | 7 | 5.144 |
| true $d$ | | 11 | 8.756 |

### 3.5 Why the first solution found is a shortest one

* A pass with bound $k$ visits every path whose nodes all satisfy $g + h \le k$. Since $h$ never overestimates, every node on a solution of length at most $k$ satisfies that, so a pass with bound $k$ cannot miss such a solution. A failed pass therefore proves that no solution of length $\le k$ exists.
* The bound starts at $h(\text{root})$, itself a lower bound, and rises by one. When the pass with bound $k$ succeeds, the pass with bound $k - 1$ has already failed (or $k = h(\text{root})$), so the solution found has length exactly $k$, the true distance.
* The search terminates because the diameter is 11. The bound never needs to exceed 11, and the code stops there regardless.

A small trace. The scramble `R B` gives `25346712313322`, with $p = 1113$, $o = 430$, and $h = \max(2, 2) = 2$. In the bound-2 pass, `R`, `R2`, `R'` have $h = 3$ and `B`, `B2` have $h = 2$, all exceeding $g + h \le 2$. `B'` has $h = 1$, so the search descends. Below it, the B face is skipped; `R` and `R2` have $h = 1$ and are cut, and `R'` reaches $h = 0$ at $g = 2$. The answer is `B' R'`, after 2 expanded nodes and 9 generated children.

### 3.6 Host verification

[`ida.c`](https://github.com/ahhhh71828/minirubik/blob/main/ida.c) runs the gates against its own breadth-first distance table, built from the transition tables independently of the pattern databases.

| Gate | Check | Result |
| :--- | :--- | :--- |
| H1 | $h(s) \le d(s)$ for all 3,674,160 states | pass; $h = d$ for 17,108 states |
| H2 | every entry filled, solved entries 0, maxima 7 and 6, transitions in range | pass |
| H3 | the search returns a path of exactly $d(s)$ moves that reaches solved, for every state | pass, 164 s |
| H3, independent | `./ida --stream \| ./verify --solutions`, checked against an oracle derived from the upstream BFS | pass: 3,674,160 distinct states, 3 min 30 s |
| H4 | packed accessor | not applicable, no packing |

Search cost by distance, where *generated* counts children produced, each costing one heuristic evaluation:

| $d$ | States | Mean generated | Worst generated | Worst expanded |
| --: | --: | --: | --: | --: |
| 7 | 227,536 | 559 | 3,940 | 655 |
| 8 | 870,072 | 2,857 | 14,751 | 2,459 |
| 9 | 1,887,748 | 13,896 | 55,578 | 9,263 |
| 10 | 623,800 | 48,130 | 250,318 | 41,720 |
| 11 | 2,644 | 206,618 | 639,792 | 106,635 |

The depth-11 search tree has 653,034,700 nodes even after same-face pruning. The heuristic cuts the worst query to 106,635 expansions, a factor of about 6,000.

### 3.7 Target measurement with the compiled C

Before writing assembly I compiled the same `ida.h` for the target, to measure the design and to obtain the reference that stage 4 must beat. [`rv32/main.c`](https://github.com/ahhhh71828/minirubik/blob/main/rv32/main.c) parses the state inlined at compile time, solves it, prints the moves through ecalls, and checks the result itself: it replays the path through the transition tables, requires the solved state, and compares the length with the expected one. [`rv32/crt0.S`](https://github.com/ahhhh71828/minirubik/blob/main/rv32/crt0.S) sets `gp` and `sp` and calls `main`. The build is

```
riscv64-elf-gcc -O2 -march=rv32i -mabi=ilp32 -ffreestanding -nostdlib -static
```

and Ripes loads the ELF (`-t elf`). Because nothing is linked in, a multiply or divide helper such as `__mulsi3` would fail to link, and the disassembly contains no `mul`, `div`, or `rem`.

| Measure | Value |
| :--- | ---: |
| `.text` | 1,432 bytes |
| `.rodata` (four tables plus strings) | 40,560 bytes |
| `21345671111111` (reference vector) | 14,057,182 retired instructions |
| Worst distance-11 state, `54721631111111` | 38,437,391 retired instructions |

I then ran every one of the 2,644 distance-11 states on `RV32_ISS` ([`rv32/sweep.sh`](https://github.com/ahhhh71828/minirubik/blob/main/rv32/sweep.sh), results in [`rv32/sweep-a0.tsv`](https://github.com/ahhhh71828/minirubik/blob/main/rv32/sweep-a0.tsv)):

| Distance-11 sweep | Value |
| :--- | ---: |
| States passing the in-program check | 2,644 / 2,644 |
| Maximum | 38,437,391 (77% of the limit) |
| Minimum | 8,468,528 |
| Mean | 12,420,971 |
| States at or above $5 \times 10^7$ | 0 |
| Wall-clock, 6 Ripes processes in parallel | 5 min 37 s |

The state the host ranked most expensive is also the most expensive on the target. The cost spreads by a factor of 4.5 across the 2,644 states, which confirms that a single sampled state says little about the worst case.

### 3.8 What this leads to

The compiled design already meets both graded thresholds, so stages 3 and 4 are about cost rather than feasibility. Dividing retired instructions by generated children gives about 60 instructions per child for the compiled C, on both the reference vector and the worst state. With roughly 640,000 children on the worst query, every instruction removed from the per-child path saves about 0.64 million instructions. Stage 3 breaks that per-child cost down and removes what the target cannot do cheaply; stage 4 rewrites the loop by hand and measures each step against this reference.

## AI Usage Disclosure

Draft, to be completed at submission (AI Guidelines §4.1). Tools: Claude Code and Codex.

* Explanation of the assignment, the baseline code, and background on heuristic search; a literature survey of candidate techniques. Design choices for stages 2–4 are recorded in later revisions together with which AI suggestions were accepted, rejected, or modified.
* The stage 1 harnesses in `stage1/` were written by Claude. Every number in §2.3 was produced by running them myself.
* The English text of this revision was drafted by Claude from my measurements and our discussion, then reviewed by me.
