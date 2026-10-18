# v4 Pipeline Screenshot Index

Captured: 2026-10-18. Operator: Codex through Ripes GUI, at the user's request. This is AI-operated visual evidence; it is not the speaker's own measurement run. Existing v3 report figures remain unchanged.

## Reproduction and provenance

- Ripes: `Ripes-v2.2.6-106-g5b8a616-mac-universal2.app`.
- Processor: 5-stage processor with forwarding and hazard detection; RV32I, M/C disabled. Processor diagram displays signal values. Layout was not independently confirmed as Extended.
- Input: `25346712313322`; `EXPECT=2`, `RENDER=0`; default v4 switches.
- `rv32/solver.S` SHA-256: `4ce3dd62675b48772e0af59172b504a917601f3b1a134785ad603914891334b4`.
- Preprocessed `solver-v4-short.s` SHA-256: `7e87146381145a8479f8e596697a3f1ea3da8ff6c81cb042687f79b6f9c747a3`.
- Generated with `riscv64-elf-gcc -E -P -x assembler-with-cpp -Irv32/build -DSTATE='"25346712313322"' -DEXPECT=2 -DRENDER=0 rv32/solver.S`.
- Full PNGs: 3024 × 1898. Use `*-crop.png` in slides: the toolbar and cursor are outside the crops. No signal values or pixels within the retained regions were altered.
- Figures 01–08 crop box `(350,195,2260,1070)` → 1910 × 875; Figure 09 box `(180,1150,2450,1530)` → 2270 × 380. Coordinates use the full PNG, `(left,top,right,bottom)`.
- Ripes returned to the start during the first attempt. Final images were all recaptured on the subsequent run. UI binding was rebuilt with the running app's bundle ID `com.yourcompany.Ripes`.

## Captures

| Figure | Instruction / observation | Cycle | Retired | Cropped file |
|---|---|---:|---:|---|
| 01 | `lhu a2,0(t0)` at 0x1f8, EX | 612 | 467 | [Load EX](fig01-load-ex-cycle612-crop.png) |
| 02 | Same load, MEM | 613 | 468 | [Load MEM](fig02-load-mem-cycle613-crop.png) |
| 03 | Same load, WB | 614 | 469 | [Load WB](fig03-load-wb-cycle614-crop.png) |
| 04 | `bltu a6,t0,next` at 0x220, EX | 624 | 477 | [Taken branch](fig04-branch-ex-cycle624-crop.png) |
| 05 | Target instruction in IF; ID/EX flushed | 625 | 477 | [Flush](fig05-branch-flush-cycle625-crop.png) |
| 06 | `sw s9,0(s7)` at 0x25c, EX | 771 | 582 | [Store EX](fig06-store-ex-cycle771-crop.png) |
| 07 | Same store, MEM | 772 | 583 | [Store MEM](fig07-store-mem-cycle772-crop.png) |
| 08 | Same store, WB; next store now in MEM | 773 | 584 | [Store WB](fig08-store-wb-cycle773-crop.png) |
| 09 | `B' R'`, `OK length 2`, exit 0 | 1045 | 779 | [Complete run](fig09-short-pass-cycle1045-crop.png) |

## Values visible in this run

- Load EX: forwarding selection `MemStage`; ALU address `0x10000a52`. MEM: read result `0x00001ab2`, write enable 0. WB: rd=x12, data `0x00001ab2`, register write enable 1. The upper-stage label can be used to identify the instruction; register pane updates occur across clock boundaries.
- Branch EX: `1 < 3`, Branch taken=1; `0x220 + 0x8c = 0x2ac`. Next cycle: target `addi a1,a1,1` in IF, two `nop (flush)` bubbles in ID/EX.
- Store EX: ALU address `0x10000020`, store data `0x10002900`. MEM: same address/data, write enable 1. WB: register write enable 0. At cycle 773 the MEM signals belong to `sw s10,4(s7)`, not the first store.
- The retained final store images show processor signals, not memory-table before/after snapshots. Do not label Figure 08 as direct evidence that a displayed memory word changed. The attempted memory-selector setup was lost when Ripes returned to the start.
- Complete run: 1045 cycles, 779 retired instructions, CPI 1.34, IPC 0.745. Clock rate shown is interactive stepping speed, not solver wall-clock performance.

## v3 and v4

The hardware pipeline is unchanged: IF, ID, EX, MEM, WB. v4 changes the program's repeated work. These first load/branch stage cycles match the earlier v3 observations. The shown branch target is `0x2ac` instead of `0x2b4`; the shown store's stages are one cycle earlier (v3 EX/MEM/WB 772/773/774; v4 771/772/773). Short-run completion is 1048/782 in the old v3 GUI evidence versus 1045/779 in this v4 run. These particular offsets are input-specific, not a constant speedup for every instruction or query.
