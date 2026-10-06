# Assignment 1: Optimizations and RISC-V Assembly

Contributed by ahhhh71828 · Fork: [ahhhh71828/minirubik](https://github.com/ahhhh71828/minirubik) · Baseline: upstream commit [`231796c`](https://github.com/sysprog21/minirubik/commit/231796c)

| Item | Value |
| :--- | :--- |
| Ripes | continuous build `v2.2.6-106-g5b8a616` (2026-08-18), macOS universal2 |
| RISC-V toolchain | Homebrew `riscv64-elf-gcc` 16.2.0 (prefix differs from `riscv64-unknown-elf-gcc`) |
| Host | Apple M5, 32 GB RAM, macOS |
| Retired instructions | `--iret` on the Ripes build above, same input, renderer compiled out |
| Code size | bytes of linked `.text`, renderer compiled out (defined now, reported from stage 4) |

> Status: this revision covers the state-space model and stages 1 to 4, including the LED matrix renderer. The pipeline walkthrough and the final cross-model checks follow in the next revision.

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
| H3 | the search returns a path of exactly $d(s)$ moves that reaches solved, for every state | pass, 163.5 s |
| H3, independent | `./ida --stream \| ./verify --solutions`, checked against an oracle derived from the upstream BFS | pass: 3,674,160 distinct states, 2 min 47 s |
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

## 4. Stage 3: Making the Search Cheap on the Target

Stage 2 ended with about 60 retired instructions per generated child. Stage 3 asks where the rest of the budget can go: fewer children, or cheaper ones. I changed one factor at a time, kept every variant buildable, and measured each on all 2,644 distance-11 states with the compiled C (GCC 16.2.0, `-O2 -march=rv32i -mabi=ilp32`). All variants share [`ida.h`](https://github.com/ahhhh71828/minirubik/blob/main/ida.h) and [`ida_perimeter.h`](https://github.com/ahhhh71828/minirubik/blob/main/ida_perimeter.h) with the host gates.

### 4.1 Fewer children: a perimeter around the goal

The A0 heuristic averages 5.144 against a true mean distance of 8.756. Most of the search is spent in the last levels of each pass, where the bound is nearly used up and the heuristic is too weak to cut. A perimeter closes that gap from the other end.

Every state within radius $r = 5$ of solved, 12,224 of them, is stored with its exact distance. The search then evaluates

$$h(s) = \begin{cases} h_0(s) & h_0(s) > r \\ d(s) & s \text{ in the perimeter} \\ r + 1 & \text{otherwise.} \end{cases}$$

where $h_0$ is A0's heuristic. It never overestimates:
* $h_0 > r$ already rules out membership, and $h_0$ is admissible.
* Inside the perimeter the value is exact.
* Outside it, every state within $r$ moves is listed, so the true distance is at least $r + 1$.

The table holds distances only, and it is the precomputed heuristic table the assignment's precomputation rule allows ("transition tables and any heuristic tables may be generated on the host"). It covers 0.33% of the state space, not the complete table the rule forbids. Every move of every solution, including the last steps through the perimeter, is found by the search on the target. This is my reading of the rule; I flag it rather than assume it.

Mean $h$ rises from 5.144 to 6.005, and the worst distance-11 query drops from 639,792 generated children to 57,157, a factor of 11.

### 4.2 Variants, one change each

| Variant | Change | Worst generated | GCC worst | GCC mean | Per child, worst |
| :--- | :--- | ---: | ---: | ---: | ---: |
| A0 | §3 | 639,792 | 38,437,391 | 12,420,971 | 60 |
| A3T | perimeter as sorted 32-bit entries, binary search; a hit appends stored moves | 57,157 | 10,347,119 | 3,515,002 | 181 |
| A3D | as A3T, distances only; IDA* searches on to solved | 57,168 | 10,377,121 | 3,527,482 | 182 |
| A3B | as A3D, entries bucketed by permutation rank | 57,168 | 4,319,765 | 1,487,587 | 76 |
| A3F | as A3B, lookup only with at most $r$ moves left | 57,168 | 4,464,307 | 1,532,112 | 78 |
| A3BX | as A3B, tables pre-scaled to byte offsets | 57,168 | **4,068,454** | **1,398,474** | 71 |

*Per child* is worst-case retired instructions divided by worst-case generated children. All six pass H1, H2/H4, and the full-domain H3 on the host. All six also pass the in-program check on all 2,644 distance-11 states on the target, with every printed path confirmed by the independent verifier. The worst state is `54721631111111` throughout.

**A0 → A3T.** Eleven times fewer children but only 3.7 times fewer instructions. Each child now costs 181 instructions instead of 60, and the difference is the lookup: a binary search over 12,224 entries takes about 14 probes, each an address computation, a load, a shift, and a branch. Fewer children had moved the bottleneck into the table.

**A3T → A3D.** Without stored moves, a child inside the perimeter with $g + d \le$ bound has exactly one kind of child that survives, one step closer, so the pass walks straight down to solved. The host count rises by only 11 children (0.02%), and the target cost by 0.3%: dropping the stored tail costs almost nothing. An earlier build of A3T, before the variants shared one lookup interface, measured 8% faster than A3D. The algorithm was the same, so that gap came from how the old interface compiled, not from search work, and I report the current build.

**A3D → A3B.** Bucketing replaces the binary search:
* `perimeter_offset[p]` (5,041 halfwords) marks where the bucket of permutation rank $p$ starts in `perimeter_entry`.
* Each entry is a 16-bit `ori << 3 | distance`, ascending.
* Only 3,751 permutations have any perimeter state. A non-empty bucket holds 3.26 entries on average and 11 at most, and a scan stops at the first ori not below the target.

The perimeter shrinks from 48,896 to 34,530 bytes, and the cost per child falls from 182 to 76. This is the step that turned the perimeter into a net win of 8.9× over A0 on the worst state.

**A3B → A3F, a negative result.** With $m$ moves left below a child, a lookup can only matter if $m \le r$: the perimeter raises $h$ to at most $r + 1$, so with more moves left the child descends whatever the lookup says. Skipping those lookups leaves the search tree unchanged, and the host streams of A3B and A3F match byte for byte over all 3,674,160 states. Yet the compiled code got 3.3% slower: the skip spares only the lookups made with more than $r$ moves left, while computing `bound - (depth + 1)` is added to every one of the 57,168 children. Stage 4 returns to this.

**A3B → A3BX.** Every rank is stored pre-scaled by 2, the byte offset of its halfword entry, so indexing a halfword table needs no shift. The pattern databases are widened to halfwords so the same scaled rank indexes them. This costs 5,769 bytes and saves about 6% on the target. A3BX is the compiled C I compare the assembly against: it is the fastest compiled variant, and it uses the same tables and layout as my final assembly.

### 4.3 Memory

| Structure | A0 | A3BX |
| :--- | ---: | ---: |
| `perm_turn`, `ori_turn` | 34,614 | 34,614 |
| `perm_pdb`, `ori_pdb` | 5,769 | 11,538 |
| `perimeter_offset`, `perimeter_entry` | — | 34,530 |
| Linked `.rodata` | 40,560 | 80,860 |
| Share of the 128 KiB budget | 30.9% | 61.7% |

## 5. Stage 4: Hand-Written RV32I

### 5.1 Build and measurement conventions

[`rv32/solver.S`](https://github.com/ahhhh71828/minirubik/blob/main/rv32/solver.S) is written by hand against the RV32I base ISA. Ripes' built-in assembler does not support `.if` or `.rodata`, so the assemble-time switches are C-preprocessor `#if`, resolved by `riscv64-elf-gcc -E -P -x assembler-with-cpp`. Ripes then assembles the result itself (`-t asm`); [`rv32/asm.sh`](https://github.com/ahhhh71828/minirubik/blob/main/rv32/asm.sh) wraps both steps.

The source uses only the subset both Ripes and GNU as accept, so the same preprocessed text also links with GNU as (`-Wl,--no-relax`) to measure section sizes. Both builds retire exactly the same number of instructions on every input I compared, so their pseudo-instruction expansions agree. The tables come from [`gen_tables.c`](https://github.com/ahhhh71828/minirubik/blob/main/gen_tables.c) as `.half` data. All halfword tables come first in `.data`, so no alignment directive is needed.

* **Retired instructions:** `--iret` on `RV32_ISS`, renderer compiled out, same input.
* **Code size:** linked `.text` bytes with the renderer compiled out.

### 5.2 Design: the current depth lives in registers

| Register | Role |
| :--- | :--- |
| `s0`–`s5` | table bases |
| `s6` | the bound |
| `s7`, `s8` | current and depth-0 frame |
| `s9`, `s10` | the rows of the face being tried |
| `s11` | the row stride |
| `a0`, `a1` | face and turn count |
| `a2`, `a3` | the latest child |
| `a4`, `a5` | the current node |
| `a6` | moves left below the child |
| `a7` | the face that led here |
| `gp`, `tp` | the root |

A 32-byte frame per depth, addressed by stepping `s7` by 32, holds only what backtracking needs: the two row pointers, the node, the child that was descended into, the face, and the turn count. It is written only on a descent. The solution is never stored separately: on success it is read back from each frame's face and turn count.

The child loop is three table loads and a maximum:

```asm
child:
    add  t0, s9, a2             # a2 holds perm rank * 2
    lhu  a2, 0(t0)
    add  t1, s10, a3
    lhu  a3, 0(t1)
    add  t0, s2, a2
    lhu  t0, 0(t0)              # perm_pdb
    add  t1, s3, a3
    lhu  t1, 0(t1)              # ori_pdb
    bgeu t0, t1, child_h
    mv   t0, t1
```

`a6` replaces the bound test: `bltu a6, t0, next` prunes when $h$ exceeds the moves left. It is decremented on descent and incremented on backtrack, never recomputed. Changing face adds the row stride to `s9` and `s10`, so there is no multiply anywhere in the search.

### 5.3 Iterative refinement

| Version | Change | Worst | Mean | `21345671111111` | `.text` |
| :--- | :--- | ---: | ---: | ---: | ---: |
| v0 (eager) | A3B's algorithm by hand; lookup whenever $h_0 \le r$ | 2,466,956 | | 893,520 | 1,148 |
| v1 | look up only with at most $r$ moves left | 2,312,468 | 798,701 | 838,622 | 1,148 |
| v2a | v1 with the lookup inlined | 2,154,801 | 745,162 | 781,691 | 1,200 |
| v2b | v2a with byte-offset tables | **1,939,773** | **672,088** | **703,904** | **1,160** |

Every version is reproducible from the final source through switches:
* v1 is `-DINDEX_TABLES -DCALL_LOOKUP`.
* v2a is `-DINDEX_TABLES`.
* v0 adds `-DEAGER_LOOKUP` to v1.

v1, v2a, and v2b each passed the in-program check on all 2,644 distance-11 states, and the independent verifier confirmed all 2,644 printed paths for each.

**v0 → v1, the same shortcut that failed in C.** In assembly, the moves left are already in `a6`, so the test costs two instructions (`li`, `bgeu`) and saves 6.3% on the worst state. In C, the same test lost 3.3%, because the count had to be rebuilt from `bound` and `depth` for every child. The algorithm is identical; what differs is that I can keep a value in a register for the whole search, which GCC did not.

**v1 → v2a, inline lookup.** Beyond saving `jal` and `ret`, inlining lets one comparison handle every outcome. The lookup only runs with at most $r = 5$ moves left. An absent state would score $r + 1 = 6$, and scanning past the target ori leaves `entry - (o << 3)` at 8 or more. Both exceed the moves left, so the single `bltu a6, t0, next` that rejects a too-distant hit also rejects both misses.

**v2a → v2b, byte offsets.** Pre-scaled ranks remove two `slli` per child and three per lookup, at the cost of 5,769 bytes of wider pattern databases. Static data is 81,216 bytes, 62% of the budget.

### 5.4 Against the compiled C

| | GCC A3BX | asm v2b | asm ÷ GCC |
| :--- | ---: | ---: | ---: |
| Worst distance-11 state | 4,068,454 | 1,939,773 | 0.477 |
| Mean over 2,644 states | 1,398,474 | 672,088 | 0.481 |
| `21345671111111` | 1,478,449 | 703,904 | 0.476 |
| Per-state ratio, range | | | 0.471 to 0.511 |
| States where the assembly is not faster | | | 0 |
| `.text` | 1,744 bytes | 1,160 bytes | 0.665 |

The comparison is fair in algorithm and data: A3BX uses the same tables and the same byte-offset layout. A3BX looks up eagerly because that is faster for the compiled C; the lazy variant A3FX measured 4,294,714 on the worst state. The worst query uses 3.9% of the $5 \times 10^7$ budget.

The assembly wins in four places:
* The current depth stays in registers, while the compiled C writes each child to its frame arrays.
* The moves-left counter replaces a per-child recomputation.
* The lookup is skipped when it cannot prune and is inlined when it can.
* Row pointers step by addition instead of two-dimensional indexing.

The one place the assembly does not win is the solved state: 542 against 475 retired instructions. Parsing and ranking run once per query, and I wrote them as plain loops. The rank's varying radix is applied by repeated addition. That costs a few dozen instructions on every query, under 0.01% of a hard one.

As a fallback that needs no perimeter, the same source built with `-DHEURISTIC_A0` solves the worst state in 14,694,457 instructions and the reference in 5,374,645, with 1,024 bytes of `.text`. That is 38% of the compiled A0.

### 5.5 Tests inside the program

The input is the 14-character string inlined at assembly time. After solving, the program does three things:
1. It prints the moves.
2. It replays them through the transition tables from the root, requiring the solved state.
3. If an expected length is given at assembly time, it compares the length with it.

It then prints `OK length n` or `FAIL length n`.

| Test | Length | Retired (`RV32_ISS`) |
| :--- | ---: | ---: |
| `12345671111111` (solved) | 0 | 542 |
| `25346712313322` (`R B`) | 2 | 866 |
| `62345713133111` | 8 | 5,969 |
| `21345671111111` (distance 11) | 11 | 703,904 |
| `54721631111111` (hardest for every variant) | 11 | 1,939,773 |

> Pending for the next revision: the same tests on a pipelined model (T7), and the pipeline walkthrough.

## 6. LED Matrix Rendering

With `RENDER=1` the same source adds a renderer for the 35×25 LED matrix. The CLI build, the one measured above, defines `RENDER=0`, which removes the renderer's code, data, delay loops, and every reference to the peripheral symbols. The two builds differ only in the renderer.

The renderer keeps its own eight-corner arrays, a cubie and a twist per position with the fixed corner at position 0. The search's two ranks suit table lookup, but drawing needs to know which cubie sits where. The flow is:
1. Before playback, it decodes the input and draws the initial state.
2. For each move in the solver's `moves[]`, it applies the move to these arrays and redraws.

A quarter turn writes into separate next-state arrays and copies them back, because a cycle updated in place would read values it had already moved. Every frame comes from the move list the search produced.

The layout is the unfolded net with U above F, the row L F R B, and D below F:
* Each facelet is 4×3 pixels.
* Each face is 2×2 facelets, so 8×6 pixels.
* One dark pixel separates neighbouring faces.
* The bounding box is 35×20, placed from row 2.

A sticker's colour is looked up from the cubie's three home faces, rotated by its twist: `basis[cubie][(slot + twist) mod 3]`. Since the sum is at most 4, the modulo is one conditional subtract. Pixels are addressed row-major from the peripheral symbols, as `LED_MATRIX_0_BASE + 4 * (y * LED_MATRIX_0_WIDTH + x)`. The clear loop runs over `LED_MATRIX_0_WIDTH` and `LED_MATRIX_0_HEIGHT`, so no address is hard-coded. The six colours are white, red, green, yellow, orange, and blue for U, R, F, D, L, and B.

At the end, the renderer requires its own corner arrays to be solved, a second check independent of the rank replay. On the GUI build of `21345671111111`, Ripes plays the initial state and each of the 11 moves, ends on six uniform faces, and prints `OK length 11`.

## AI Usage Disclosure

Draft, to be completed at submission (AI Guidelines §4.1). Tools: Claude Code and Codex.

* Explanation of the assignment, the baseline code, and background on heuristic search; a literature survey of candidate techniques. Design choices for stages 2–4 are recorded in later revisions together with which AI suggestions were accepted, rejected, or modified.
* The stage 1 harnesses in `stage1/` were written by Claude. Every number in §2.3 was produced by running them myself.
* The English text of this revision was drafted by Claude from my measurements and our discussion, then reviewed by me.
