# VCC Execution Engine Redesign: Event-Driven Block Execution with JIT

## Overview

This document describes a staged architectural redesign of VCC's CPU execution
engine. The goal is to replace the current per-instruction interpreter loop and
hand-coded event dispatch with an event-heap-driven block execution model that
supports optional JIT compilation of hot blocks, while maintaining
cycle-accurate fidelity for games and offering a turbo mode for non-interactive
workloads.

The redesign is incremental. Each stage produces a working, improved VCC. No
stage depends on a later stage being completed.


## Current Architecture

### Execution Model

The current execution is scanline-driven:

1. `RenderFrame()` (coco3.cpp:192) drives one frame of 262 scanlines at ~60 Hz.

2. Each scanline calls `HLINE()` (coco3.cpp:391) which:
   - Calls `UpdateAudio()`.
   - Calls `CPUCycle()` for the non-HSYNC portion of the line (~58,613 ns).
   - Asserts HSYNC low, calls `PakTimer()`.
   - Calls `CPUCycle()` for the HSYNC width (~5,000 ns).
   - Asserts HSYNC high.

3. `CPUCycle()` (coco3.cpp:409) converts nanoseconds to CPU cycles, then enters
   a state machine that interleaves CPU execution with timer interrupts and
   audio sample collection. This is a four-way switch (`StateSwitch`) with
   nested comparisons when both events are pending.

4. `CPUExec(cycles)` (mc6809.cpp:333) runs a loop:
   - Check debugger halt/step/breakpoints (linear vector search per instruction).
   - Check trace capture.
   - Latch and check interrupts.
   - Execute one instruction via `Do_Opcode()`.
   - Update joystick ramp timer.
   - Repeat until cycle budget exhausted.

5. `Do_Opcode()` is a ~2300-line switch statement over the opcode byte. Page 2
   (0x10 prefix) and Page 3 (0x11 prefix) are separate switch functions.

### Memory Access

Every instruction fetch and data access calls `MemRead8()` (tcc1014mmu.cpp:239):

```
if (address < 0xFE00):
    page = MmuRegisters[MmuState][address >> 13]
    if MemPageOffsets[page] == 1:           // RAM
        return MemPages[page][address & 0x1FFF]
    else:                                    // Cartridge ROM
        return PackMem8Read(offset + (address & 0x1FFF))   // mutex!
if (address > 0xFEFF):
    return port_read(address)               // I/O
// 0xFE00-0xFEFF: vector area, conditional on RamVectors flag
```

`MemRead16()` calls `MemRead8()` twice. Each `PackMem8Read()` acquires a mutex.

The MMU lookup expression `MmuRegisters[MmuState][address >> 13]` is evaluated
redundantly -- once for the `MemPageOffsets` check and again for `MemPages`.

### Interrupt Generation

Three GIME interrupt sources:
- **HSYNC**: asserted when HSYNC goes low, every scanline.
- **VSYNC**: asserted when VSYNC goes low, once per frame.
- **Timer**: asserted from `CPUCycle()` when `NanosToInterrupt` expires.

Each can route to FIRQ or IRQ based on GIME registers 0x92/0x93.
Cartridge and NMI interrupts come from the pak interface.

### Timing Constants

- COLORBURST = 3,579,545 Hz
- CPU clock = COLORBURST / 4 = 894,886.25 Hz (~1.8 MHz)
- Lines per second = 60 * 262 = 15,720
- Nanoseconds per line = 63,613.23 ns
- Cycles per line = ~56.9 (at 1x clock)

### Problems

1. **Per-instruction overhead**: Debugger checks, trace checks, interrupt
   latching, and joystick timer all execute per instruction even in the
   common case where none are active.

2. **Event interleaving is hand-coded**: The `StateSwitch` logic handles 2
   event sources (timer + audio) with 8 code paths. Adding a new timed
   event means editing this mess. The nested if/else for simultaneous
   events is fragile.

3. **MemRead8 is expensive for instruction fetch**: The opcode fetch path
   goes through range checks, MMU lookup, ROM/RAM discrimination, and
   potentially a mutex-protected cartridge read. The common case (fetching
   from RAM) pays for all the uncommon cases.

4. **No block-level execution**: Every instruction is dispatched individually.
   There's no amortization of loop overhead across sequences of instructions
   that don't interact with hardware events.

5. **Overclock is a simple multiplier**: `OverClock` scales cycles linearly
   but doesn't change the event model. At high overclock values, the CPU
   runs many more instructions between events but still checks
   interrupts/debugger per instruction.

6. **Turbo mode doesn't exist**: There's no way to decouple emulated time
   from wall-clock time for batch workloads.


## Proposed Architecture

### Core Concept: Event Heap + Block Execution

Replace the hand-coded event interleaving with a min-heap of scheduled events.
Replace per-instruction dispatch with block execution where the cycle cost of
a block is checked against the next event deadline.

```
MasterCycle = 0

EventHeap:
    { cycle: 1200,  handler: hsync_low,      rearm: +3612 }
    { cycle: 3500,  handler: audio_sample,    rearm: +22 }
    { cycle: 3612,  handler: timer_interrupt, rearm: +MasterTickCounter }
    ...

loop:
    deadline = EventHeap.top().cycle
    block = get_block(PC, mmu_state)

    if MasterCycle + block.cycles <= deadline:
        execute(block)                  // JIT or interpreted
        MasterCycle += block.cycles
    else:
        // Block straddles an event boundary.
        // Fall back to interpreter, instruction by instruction,
        // until MasterCycle >= deadline.
        interpret_until(deadline)

    while MasterCycle >= EventHeap.top().cycle:
        event = EventHeap.pop()
        event.handler()
        event.cycle += event.rearm     // reschedule
        EventHeap.push(event)

    if faithful_mode:
        throttle_to_wall_clock(MasterCycle)
```

### Event Sources

All current timed events become heap entries:

| Event           | Period (cycles @ 1x) | Notes                          |
|-----------------|----------------------|--------------------------------|
| HSYNC low       | ~57 per line         | Triggers GIME HSYNC IRQ/FIRQ   |
| HSYNC high      | ~7 per line          | End of blanking                |
| VSYNC low       | every 262 lines      | Triggers GIME VSYNC IRQ/FIRQ   |
| VSYNC high      | 4 lines after low    | End of vertical blank          |
| GIME timer      | configurable         | From GIME registers            |
| Audio sample    | ~20 cycles @ 44.1kHz | Calls GetDACSample()           |
| Joystick ramp   | varies               | Comparator timing              |

New peripherals register events by inserting into the heap. The CPU engine
doesn't know or care what the events are.

### Block Model

A **block** is a sequence of instructions with a known aggregate cycle cost
and known memory footprint. A block ends at:

- A branch (conditional or unconditional).
- A jump or call (JSR, BSR, JMP, RTS, RTI).
- A SWI/SWI2/SWI3 (OS-9 system call -- next byte is data, not an opcode).
- A store to a location that could be executable (self-modification concern).
- A write to GIME/MMU registers (changes memory map).
- An instruction that modifies the stack pointer in a way that affects
  interrupt behavior (CWAI, SYNC).
- A configurable maximum block size (for bounding interpreter fallback cost).

The cycle cost of a block is the sum of the cycle costs of its instructions
(including addressing mode costs from `CalculateEA`). For indexed modes with
indirect addressing, the cost depends on the postbyte, which is static within
a block.

### Block Cache

Blocks are keyed by **physical address + MMU state hash**. The physical address
is computed from `MmuRegisters[MmuState][PC >> 13]` and the page offset. This
ensures that the same logical PC under different MMU mappings produces
different blocks.

The cache maps: `(physical_page, page_offset) -> Block`

A block contains:
- Physical address range it covers.
- Aggregate cycle cost.
- List of memory pages it reads from (for invalidation).
- Optional: JIT-compiled native code pointer.
- Hot count (number of times entered).

### Block Invalidation

Memory writes must check whether the written address falls within a cached
block. The mechanism:

1. Maintain a **block presence bitmap**: one bit per 256-byte region of
   physical memory. If the bit is set, at least one cached block includes
   bytes from that region.

2. On `MemWrite8`, after the normal write, check the bitmap for the written
   physical address. If the bit is set, walk the block list for that region
   and invalidate any block whose address range includes the written address.

3. On MMU register write, invalidate all blocks whose physical pages are
   affected by the mapping change. Since MMU writes are rare (mostly OS-9
   context switches), this is acceptable.

At 8MB physical with 256-byte granularity, the bitmap is 32KB. With 8KB
granularity (matching MMU pages), it's 1024 bits = 128 bytes. The right
granularity depends on how many false positives we're willing to tolerate.
Start coarse (8KB), refine if needed.

### ROM Optimization

ROM pages are known to be immutable. Blocks residing entirely in ROM are
never invalidated by memory writes. They're prime candidates for early JIT
compilation and can be pre-analyzed at ROM load time to identify entry
points (including Microsoft's mid-instruction jump targets, which can be
discovered by tracing all reachable paths from known vectors).

The 32KB of CoCo 3 ROM (coco3.rom) and the 8KB disk ROM (disk11.rom)
together contain the hottest code in the system during BASIC operation.
Pre-JITing these gives immediate benefit.

### JIT Compilation

JIT is optional and layered on top of the block model. The interpreter
handles all blocks by default. JIT compilation is triggered when a block's
hot count exceeds a threshold (initially 3).

The JIT translates 6809/6309 instructions to x86-64 native code. Each
translated block:

- Reads guest memory through a fast path (direct pointer into `MemPages[]`
  for RAM, no MMU lookup per access since the physical page is known at
  translation time).
- Writes guest memory through a checked path (must trigger invalidation).
- Updates the master cycle counter by the block's cost on exit.
- Returns to the dispatcher on block exit (branch, call, etc.).

For blocks that cross into I/O space or cartridge ROM (rare), the JIT falls
back to calling `MemRead8`/`MemWrite8`.

JIT is not required for correctness. The interpreter is always available as
fallback. A block can transition between interpreted and JIT-compiled based on
invalidation and re-heating.

### Self-Modifying Code and the Cassette Loader Pattern

The classic pattern: a loader writes code into RAM which has the side effect
of modifying a vector that an active IRQ uses. On the next IRQ, execution
jumps into the new code. There is a window where this method fails, but you
just rewind the tape and try again.

In this architecture:

1. The loader's stores to RAM trigger bitmap checks. If no block is cached
   at those addresses (likely, since the code was just written), no
   invalidation is needed.

2. The store to the IRQ vector address is a write to low memory. If a block
   has been cached that reads the vector (e.g., the IRQ dispatch sequence),
   it gets invalidated.

3. On the next IRQ, the dispatcher finds no cached block at the new target.
   It falls back to interpretation, building a new block entry. After a few
   executions, the new block gets JIT-compiled.

This is correct by construction. No special-casing needed.

### Jump-Into-Middle-of-Instruction (Microsoft ROM Trick)

Microsoft's ROMs use a trick where a branch jumps 1 byte into a 2-byte
instruction, reinterpreting the operand byte as an opcode. Example:

```
1000: LDA #$21    ; opcode 86, operand 21
1002: BRA ...
...
somewhere: BRA $1001  ; jumps to 1001, which decodes as BHI (opcode 21)
```

This means the same physical bytes produce different blocks depending on the
entry point. The block cache handles this naturally because blocks are keyed
by their start address. Address 0x1000 and 0x1001 produce different cache
entries with different decoded instruction sequences.

Pre-analysis of ROM can discover these alternate entry points by tracing all
reachable paths from vectors and known entry points. Any address that is a
branch target becomes a potential block start.

### Faithful vs. Turbo Mode

A single flag controls the mode:

- **Faithful mode**: After each block execution, compare `MasterCycle` (scaled
  to wall-clock nanoseconds) against the host clock. If ahead, yield/sleep
  until wall-clock catches up. Events fire at the correct real-time moments.
  Audio plays at normal speed. Games work.

- **Turbo mode**: Never wait for wall-clock. Execute blocks as fast as the
  host CPU allows. Events still fire at the correct *relative* cycle
  boundaries, so OS-9 tick counting and process scheduling work. Audio is
  either disabled or fast-forwarded. Video renders frames but skips
  wall-clock sync.

  Turbo mode is useful for: compiling code under OS-9, running batch
  programs, booting NitrOS-9, Basic09 development cycles, and running
  financial math at full host speed.

The event heap is identical in both modes. The only difference is the
presence or absence of the throttle call after block execution.


## Staged Implementation

### Stage 1: Event Heap

**Replace the `CPUCycle()` state machine with an event heap.**

- Define an `Event` struct: `{ uint64_t cycle; void (*handler)(); uint64_t rearm_delta; }`.
- Implement a small min-heap (std::priority_queue or hand-rolled for ~8 entries).
- Register HSYNC, VSYNC, timer, and audio sample events at initialization.
- `CPUCycle()` becomes: convert nanos to cycle deadline, call `CPUExec()` up to
  that deadline, fire any events whose cycle has passed, reschedule them.
- `CPUExec()` is unchanged internally -- still per-instruction interpretation.
- Verify: all existing behavior preserved. Games, audio, interrupts all work.

**Deliverable**: Cleaner code, easier to add new timed events. No performance
change expected. Foundation for everything else.

### Stage 2: Deferred Debugger Checks

**Move debugger overhead out of the hot path.**

- Add a single `bool debugger_active` flag, set when any breakpoint or trace
  is enabled.
- Gate all debugger checks in `MC6809Exec()` behind this flag. When false
  (the common case), the per-instruction cost drops to: latch interrupts,
  check interrupt flags, execute opcode, advance cycle counter.
- The breakpoint check currently does `std::find()` on a vector. Replace with
  an `std::unordered_set` or a 64KB bitmap if breakpoints are by-address.

**Deliverable**: Measurably lower host CPU usage during normal execution.

### Stage 3: MemRead8 Fast Path

**Optimize the instruction fetch path.**

- Add `MemFetch8(pc)`: assumes address < 0xFE00 and the page is RAM
  (`MemPageOffsets[page] == 1`). Single array lookup, no branches in the
  common case. Falls back to full `MemRead8()` if assumptions fail.
- Cache the current PC page pointer: when PC crosses an 8KB boundary,
  recompute `current_page_ptr = MemPages[MmuRegisters[MmuState][pc >> 13]]`.
  Subsequent fetches within the page are a single indexed load.
- Fix the redundant MMU lookup in `MemRead8` where
  `MmuRegisters[MmuState][address >> 13]` is computed twice.

**Deliverable**: Fewer memory indirections per instruction fetch. Meaningful
for high overclock settings.

### Stage 4: Turbo Mode

**Add the wall-clock throttle bypass.**

- Add a `turbo` flag to the emulator state.
- When turbo is set, skip the frame-rate throttle in `RenderFrame()`.
  Optionally skip audio output (silence the buffer or don't submit it).
- Wire it to a menu option or hotkey.

**Deliverable**: Immediate practical value for OS-9 development workflows.
OS-9 boots in seconds. Compilations finish instantly. Basic09 programs
run at host speed.

### Stage 5: Block Detection and Costing

**Identify basic blocks and compute their aggregate cycle cost.**

- At the start of `CPUExec()`, scan forward from PC to identify the current
  block: decode instructions without executing them until hitting a block
  terminator (branch, jump, SWI, etc.) or a maximum block size.
- Sum the cycle costs of all instructions in the block.
- Cache the block keyed by physical address.
- If `MasterCycle + block.cycles <= next_event_deadline`, execute the entire
  block and advance `MasterCycle` by `block.cycles`. Otherwise, fall back to
  per-instruction interpretation until the deadline.

- The block scan is itself cached, so the cost is paid once per unique block.
  Subsequent visits are a cache lookup.

**Deliverable**: Reduced dispatch overhead for straight-line code. The cycle
counter advances once per block instead of once per instruction.

### Stage 6: Write Tracking and Block Invalidation

**Detect when cached blocks are invalidated by memory writes.**

- Add the block presence bitmap (one bit per 8KB physical page initially).
- On `MemWrite8`, check the bitmap. If set, scan for and invalidate
  affected blocks.
- On MMU register writes, invalidate blocks for affected mappings.
- Mark ROM pages as non-invalidatable.

**Deliverable**: Correctness guarantee for the block cache in the presence
of self-modifying code and MMU changes. Required foundation for JIT.

### Stage 7: JIT Compilation

**Translate hot blocks to x86-64 native code.**

- When a block's execution count exceeds the hot threshold, translate it.
- The JIT emits x86-64 code that:
  - Reads from a pre-resolved physical page pointer (no MMU lookup).
  - Writes through a checked path (triggers invalidation).
  - Updates CC flags using native x86 flag operations where possible.
  - Returns to the block dispatcher on exit.
- Invalidated JIT blocks are freed and re-interpreted. They may be
  re-JITed if they become hot again.

**Deliverable**: Native-speed execution of hot code paths. Primarily
benefits tight loops in ROM (BASIC interpreter, OS-9 kernel) and
user programs under turbo mode.

### Stage 8: ROM Pre-Analysis

**Ahead-of-time analysis of ROM images.**

- At ROM load time, trace all reachable paths from known vectors (RESET,
  IRQ, FIRQ, NMI, SWI, SWI2, SWI3) and from the Extended BASIC dispatch
  table.
- Identify all valid entry points, including Microsoft's mid-instruction
  jump targets.
- Pre-build (and optionally pre-JIT) blocks for all discovered entry points.
- Cache the analysis to disk so it's not repeated on every launch.

**Deliverable**: Faster startup, immediate JIT benefit for ROM code, and
a complete map of ROM execution paths for debugging and documentation.


## Risk Assessment

| Risk | Mitigation |
|------|------------|
| Cycle accuracy regression | Event heap fires at exact cycle. Block execution only used when block fits before deadline. Interpreter fallback preserves instruction-level accuracy. |
| Self-modifying code breaks | Write tracking + invalidation. Interpreter is always available. |
| MMU changes invalidate JIT | MMU writes trigger invalidation of affected blocks. Physical-address keying prevents stale mappings. |
| JIT bugs | Every JIT block can be disabled individually. Fallback to interpreter is always correct. |
| Games break in turbo mode | Turbo mode is opt-in. Faithful mode is the default. |
| Debugger interaction | Debugger active flag gates block execution. When debugging, fall back to per-instruction mode. |
| Complexity | Each stage is independently valuable and testable. Later stages can be deferred indefinitely. |


## Notes

- The HD6309 (hd6309.cpp, 7611 lines) has the same architecture as the
  MC6809 and would receive the same treatment. The block model and JIT
  would need to handle the additional 6309 instructions (TFM, inter-register
  ops, W register, etc.).

- The `PackMem8Read()` mutex is a concern for JIT. Cartridge ROM reads
  should be mutex-free for JITed blocks. Since cartridge ROM content doesn't
  change during execution, the JIT can resolve the pointer at translation
  time and read directly.

- Audio sample collection at 44.1 kHz means an audio event fires roughly
  every 20 CPU cycles at 1x clock. This is very frequent and will often
  interrupt block execution. At higher overclock, the ratio improves.
  At 100x overclock, audio fires every ~2000 cycles, which is a comfortable
  block size.

- The 6809's position-independent code style (PCR addressing) means block
  addresses are meaningful -- the same code at a different address behaves
  differently. Physical-address keying handles this correctly.
