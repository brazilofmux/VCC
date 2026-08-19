#pragma once
/*
Copyright 2025 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).

    VCC (Virtual Color Computer) is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    VCC (Virtual Color Computer) is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with VCC (Virtual Color Computer). If not, see
    <http://www.gnu.org/licenses/>.
*/

// BlockCache: Tracks basic blocks of 6809/6309 code for batch execution.
//
// A "block" is a straight-line sequence of instructions ending at a control
// flow change (branch, jump, call, return, SWI, SYNC, CWAI) or a maximum
// instruction count.
//
// Blocks are discovered dynamically: the first execution records the block,
// subsequent executions use the cached instruction count and cycle cost to
// execute the block in a tight loop without per-instruction interrupt checks.
//
// Write invalidation: A 64KB reverse map tracks which block (if any) covers
// each logical address. When memory is written, the reverse map provides O(1)
// lookup to find and invalidate the affected block. This handles self-modifying
// code, code loading, and other dynamic code scenarios.
//
// MMU changes invalidate the entire cache since logical-to-physical mappings
// have changed.

#include <cassert>
#include <cstdint>
#include <cstring>
#include "DecodedInst.h"
#include "BlockDecoder.h"

// alignas(256) rounds sizeof up so the JIT chain stub can index the
// slot array with a shift instead of a multiply (and every slot starts
// on a cache-line boundary). The table is 1MB; 14 instructions is the
// most that still fits the 256-byte stride (14*16 + 24 = 248).
struct alignas(256) CachedBlock
{
    uint16_t start_pc;      // logical PC where this block starts
    uint16_t end_pc;        // PC after the last instruction in the block
    uint8_t  num_insns;     // number of instructions in the block
    uint8_t  total_cycles;  // total cycle cost of the block
    // Executions through the interpreter dispatch path, saturating at
    // the JIT hotness threshold. Runtime-recorded blocks earn a native
    // thunk once hot; pre-populated ROM blocks are emitted eagerly and
    // never consult this. Preserved when a re-record produces an
    // identical block (see FinishRecord).
    uint8_t  exec_count;
    // Diagnostic: the thunk has no memory accesses or handler calls,
    // so VCC_VERIFY_PURE can run it and the interpreter from the same
    // snapshot and compare. Set alongside native_entry.
    uint8_t  pure_thunk;
    uint32_t generation;    // generation when this block was cached
    // Taken-direction traces: bit i set means insns[i] is a short
    // branch recorded TAKEN - execution continues at its target, and
    // replay/thunks exit the block if the runtime outcome differs.
    // Zero for purely contiguous blocks.
    uint16_t taken_mask;

    // Optional level-1 JIT thunk: a small chunk of native x86 that
    // replaces the per-instruction dispatch loop with a straight
    // sequence of __cdecl handler calls. Set by BlockJit::EmitBlock
    // after this slot is committed; nullptr means "use the interpreter
    // dispatch loop." See BlockJit.h.
    void (*native_entry)(void);

    // Pre-decoded instructions for this block. Filled by DecodeBlock()
    // when the block is first cached. Handlers are called with a pointer
    // to the corresponding DecodedInst, allowing incremental conversion
    // from memory-reading to pre-decoded operand access.
    DecodedInst insns[14];  // MAX_BLOCK_INSNS — can't forward-ref constexpr
};

// The chain stub bakes the 256-byte stride as a shift; a growing
// DecodedInst or insns[] must fail the build, not corrupt slot math.
static_assert(sizeof(CachedBlock) == 256, "CachedBlock must stay 256 bytes");

struct BlockCacheStats
{
    uint64_t block_hits;        // instructions executed via cached blocks
    uint64_t block_insns;       // total instructions in those blocks
    uint64_t single_steps;      // instructions executed one-at-a-time
    uint64_t blocks_recorded;   // new blocks committed to cache
    uint64_t record_cancels;    // in-progress recordings abandoned
    uint64_t rejected_blocks;   // decoded recordings rejected before caching
    uint64_t invalidations;     // individual block invalidations (write)
    uint64_t bulk_invalidations;// full cache invalidations (MMU change)
};

class BlockCache
{
public:
    // Cache size must be power of 2 for fast masking.
    static constexpr int CACHE_SIZE = 4096;
    static constexpr int CACHE_MASK = CACHE_SIZE - 1;

    // Maximum instructions per block. Keeps interrupt latency bounded.
    // At ~5 cycles/instruction average, 12 instructions = ~60 cycles = ~33us.
    static constexpr int MAX_BLOCK_INSNS = 14;

    // Sentinel value meaning "no block covers this address".
    static constexpr uint16_t NO_BLOCK = 0xFFFF;

    BlockCache() { Clear(); }

    void Clear()
    {
        memset(blocks_, 0, sizeof(blocks_));
        memset(reverseMap_, 0xFF, sizeof(reverseMap_));  // fill with NO_BLOCK
        memset(pageBitmap_, 0, sizeof(pageBitmap_));
        generation_ = 1;  // start at 1 so 0-initialized blocks are invalid
    }

    // Look up a block by its starting PC. Returns nullptr if not cached.
    // Is this address inside a LIVE block that starts elsewhere?
    // Recording from such an address would create an overlapping block
    // that the one-owner reverse-map invariant then kills the incumbent
    // for - churn that dismantles unrolled loops every time an
    // interrupt resumes mid-loop. The caller single-steps without
    // recording instead; a few instructions later it reaches the
    // incumbent's start and chains normally.
    bool CoveredByLiveBlock(uint16_t pc) const
    {
        const uint16_t owner = reverseMap_[pc];
        if (owner == NO_BLOCK || owner == pc)
            return false;
        const CachedBlock& b = blocks_[owner & CACHE_MASK];
        return b.generation == generation_ && b.start_pc == owner;
    }

    // Debug forensics.
    uint16_t DebugMapOwner(uint16_t addr) const { return reverseMap_[addr]; }
    int DebugPageBit(uint16_t addr) const
    { return (pageBitmap_[addr >> 11] >> ((addr >> 8) & 7)) & 1; }

    // Raw slot storage and generation counter, exposed so the JIT can
    // emit a chain stub that revalidates a slot exactly the way
    // Lookup() does (tag + generation) before jumping into its thunk.
    CachedBlock* SlotBase() { return blocks_; }
    const uint32_t* GenerationAddr() const { return &generation_; }

    const CachedBlock* Lookup(uint16_t pc) const
    {
        const CachedBlock& b = blocks_[pc & CACHE_MASK];
        if (b.generation == generation_ && b.start_pc == pc)
            return &b;
        return nullptr;
    }

    // Check if a memory write invalidates a cached block.
    // Called from MemWrite8 on every RAM write. Must be VERY fast.
    // The coarse bitmap (32 bytes, fits in L1) rejects 99.99% of writes
    // before touching the 128KB reverse map.
    // Returns true when a live block was actually invalidated by this
    // write (the recorder uses that to split self-modifying sequences).
    bool InvalidateIfCached(uint16_t address)
    {
        // Fast reject: check coarse bitmap (one bit per 256-byte page).
        // This 32-byte array stays in L1 cache. Almost all writes bail here.
        if (!(pageBitmap_[address >> 11] & (1 << ((address >> 8) & 7))))
            return false;

        // Slow path: a block exists on this page. Use reverse map.
        uint16_t block_pc = reverseMap_[address];
        if (block_pc == NO_BLOCK)
            return false;

        CachedBlock& b = blocks_[block_pc & CACHE_MASK];
        if (b.generation == generation_ && b.start_pc == block_pc)
        {
            ForEachBlockRange(b.start_pc, b.num_insns, b.insns, b.taken_mask,
                              [this, &b](uint16_t s, uint16_t e) {
                                  ReleaseReverseMap(b.start_pc, s, e);
                              });
            b.generation = 0;  // invalidate
            stats_.invalidations++;
            return true;
        }

        // Stale reverse map entry — block was already evicted or generation changed.
        reverseMap_[address] = NO_BLOCK;
        return false;
    }

    // Begin recording a new block at the given PC.
    void BeginRecord(uint16_t pc)
    {
        recording_ = true;
        rec_start_pc_ = pc;
        rec_insn_count_ = 0;
        rec_cycle_start_ = 0;  // caller sets this
        rec_taken_mask_ = 0;
    }

    // Called after each instruction during recording.
    // Returns true if the block should continue, false if it should end.
    // taken_guard: the instruction was a short branch that went its
    // taken way - the trace follows it (taken-direction traces).
    bool RecordInstruction(uint16_t next_pc, int cycle_counter,
                           bool taken_guard = false)
    {
        if (!recording_) return false;

        if (taken_guard)
            rec_taken_mask_ |= (uint16_t)(1u << rec_insn_count_);
        rec_insn_count_++;

        if (rec_insn_count_ >= MAX_BLOCK_INSNS)
        {
            // Max size reached, end the block here
            FinishRecord(next_pc, cycle_counter);
            return false;
        }
        return true;
    }

    // End recording due to a block terminator (branch, jump, etc.).
    void EndRecord(uint16_t next_pc, int cycle_counter)
    {
        if (recording_)
            FinishRecord(next_pc, cycle_counter);
    }

    // Cancel recording (e.g., if an interrupt fires mid-block).
    void CancelRecord()
    {
        if (recording_)
            stats_.record_cancels++;
        recording_ = false;
    }

    bool IsRecording() const { return recording_; }

    void SetCycleStart(int cycles) { rec_cycle_start_ = cycles; }

    // Stats: call from the execution loop
    // One read-modify-write instead of two: hits in the high word,
    // instruction count in the low word of a single accumulator. This
    // runs once per block dispatch (hundreds of millions per second on
    // hot workloads). Unpacked in GetAndResetStats; insns per interval
    // stay far below 2^32.
    void RecordBlockHit(int num_insns)
    {
        hit_accum_ += (1ull << 32) | (unsigned)num_insns;
    }
    void RecordSingleStep() { stats_.single_steps++; }

    // Return accumulated stats and reset counters.
    BlockCacheStats GetAndResetStats()
    {
        BlockCacheStats s = stats_;
        s.block_hits += hit_accum_ >> 32;
        s.block_insns += hit_accum_ & 0xFFFFFFFFull;
        hit_accum_ = 0;
        memset(&stats_, 0, sizeof(stats_));
        return s;
    }

    // Insert a fully-decoded block. Used by the ROM block pre-populator
    // to seed the cache from offline analysis results before runtime
    // recording would otherwise discover the same blocks. The caller is
    // responsible for filling in start_pc, end_pc, num_insns,
    // total_cycles, and the insns[] array; this method handles the
    // generation tag and cache slot bookkeeping.
    //
    // If a slot already holds a different valid block, the new entry
    // replaces it. Pre-population runs at reset when the cache is empty,
    // so collisions only happen when multiple pre-built blocks hash to
    // the same slot - "later block wins" is fine for that case.
    // Returns the slot the block landed in. The JIT emitter needs the
    // slot's stable address to bake &slot.insns[i] as immediates - it
    // cannot use the caller's temporary CachedBlock, which is on the
    // stack and goes away.
    CachedBlock* InsertPrebuiltBlock(const CachedBlock& block)
    {
        if (!ValidateDecodedBlock(block.start_pc, block.num_insns,
                                  block.insns, block.end_pc, block.taken_mask))
        {
            stats_.rejected_blocks++;
            return nullptr;
        }

        CachedBlock& slot = blocks_[block.start_pc & CACHE_MASK];

        // If we're replacing a stale entry from before our generation,
        // its reverse-map entries are already invisible (generation
        // mismatch); no cleanup needed.
        if (slot.generation == generation_)
            ForEachBlockRange(slot.start_pc, slot.num_insns, slot.insns,
                              slot.taken_mask,
                              [this, &slot](uint16_t s, uint16_t e) {
                                  ReleaseReverseMap(slot.start_pc, s, e);
                              });

        slot = block;
        slot.generation = generation_;
        slot.exec_count = 0;
        slot.pure_thunk = 0;

        ForEachBlockRange(slot.start_pc, slot.num_insns, slot.insns,
                          slot.taken_mask,
                          [this, &slot](uint16_t s, uint16_t e) {
                              ClaimReverseMap(slot.start_pc, s, e);
                              MarkPageBits(s, e);
                          });

        stats_.blocks_recorded++;
        return &slot;
    }

    // Invalidate a cache entry by PC.
    void Invalidate(uint16_t pc)
    {
        CachedBlock& b = blocks_[pc & CACHE_MASK];
        if (b.generation == generation_ && b.start_pc == pc)
        {
            ForEachBlockRange(b.start_pc, b.num_insns, b.insns, b.taken_mask,
                              [this, &b](uint16_t s, uint16_t e) {
                                  ReleaseReverseMap(b.start_pc, s, e);
                              });
            b.generation = 0;
            stats_.invalidations++;
        }
    }

    // Invalidate all entries (e.g., on MMU change).
    // O(1): just bump the generation counter. All existing blocks become
    // stale instantly. Reverse map and bitmap entries self-clean lazily.
    void InvalidateAll()
    {
        generation_++;
        stats_.bulk_invalidations++;
        // On the rare wraparound, do a full clear
        if (generation_ == 0)
            Clear();
    }

    // Drop every native thunk pointer. MUST be called before
    // BlockJit::Reset() reclaims the code arena - afterwards any stale
    // native_entry would jump into freshly-overwritten arena memory.
    void ClearAllNativeEntries()
    {
        for (int i = 0; i < CACHE_SIZE; i++)
        {
            blocks_[i].native_entry = nullptr;
            blocks_[i].exec_count = 0;
            blocks_[i].pure_thunk = 0;
        }
    }

private:
    CachedBlock blocks_[CACHE_SIZE];

    // Reverse map: for each address in the 64KB logical space, stores the
    // start_pc of the cached block covering that address, or NO_BLOCK.
    // This gives O(1) lookup on memory writes.
    uint16_t reverseMap_[65536];

    // Coarse page bitmap: one bit per 256-byte page (32 bytes total).
    // Indexed as pageBitmap_[address >> 11] & (1 << ((address >> 8) & 7)).
    // Fits in a single L1 cache line for fast rejection of data-page writes.
    uint8_t pageBitmap_[32];

    // Generation counter for O(1) bulk invalidation.
    // Blocks are valid only if their generation matches this value.
    uint32_t generation_ = 1;

    // Recording state
    bool recording_ = false;
    uint16_t rec_start_pc_ = 0;
    int rec_insn_count_ = 0;
    int rec_cycle_start_ = 0;
    uint16_t rec_taken_mask_ = 0;   // taken-guard bits for the recording

    // Performance counters (reset by GetAndResetStats)
    BlockCacheStats stats_ = {};
    uint64_t hit_accum_ = 0;    // (hits << 32) | insns, see RecordBlockHit

    // Set a bit in the page bitmap for the given address.
    void SetPageBit(uint16_t address)
    {
        pageBitmap_[address >> 11] |= (1 << ((address >> 8) & 7));
    }

    // Clear page bitmap bits for a block's range.
    // Note: we could leave stale bits (safe, just causes unnecessary reverse
    // map lookups). But clearing is cheap for small blocks.
    void ClearPageBitmap(uint16_t start_pc, uint16_t end_pc)
    {
        // Only clear if no other block covers the same page.
        // For simplicity, just clear — the bitmap will be rebuilt as
        // blocks are re-cached. A stale '1' bit is safe (causes a reverse
        // map check that returns NO_BLOCK).
        // Actually, don't clear — stale bits are safe and avoiding the
        // scan is faster. The bitmap gets reset on InvalidateAll().
        (void)start_pc;
        (void)end_pc;
    }

    // Mark the reverse map for a block's address range.
    // Claim [range_start, range_end) for the block whose start PC is
    // `owner` (which is NOT the range start for the secondary ranges of
    // a taken-direction trace!). INVARIANT: every live block's every
    // byte maps to that block, so a write invalidates the one true
    // owner and no stale overlapping block can survive. Traces overlap
    // freely - several trace blocks can each contain the same
    // self-modified instruction (DECB's CHRGET taught us this the hard
    // way) - so claiming an address owned by a DIFFERENT live block
    // invalidates that block; its other map entries go stale and
    // lazy-clean in InvalidateIfCached.
    void ClaimReverseMap(uint16_t owner, uint16_t range_start, uint16_t range_end)
    {
        // Overlapping blocks are a design feature (the ROM prepopulator
        // seeds blocks at every instruction boundary; superblocks
        // stagger); later claimants simply take over the map entry. An
        // eager kill-the-previous-owner variant proved correct but
        // dismantled the prepopulated fabric wholesale (-7%). The
        // residual hazard - a write invalidating only the mapped owner
        // while an unmapped overlapping block goes stale - is closed
        // for the self-modifying case by the recorder's code-write
        // split plus ReplayBlock's generation check, and bulk MMU
        // invalidations cover code reloads in practice. A per-page
        // epoch scheme would close it fully; see docs.
        for (uint16_t a = range_start; a != range_end; a++)
            reverseMap_[a] = owner;
    }

    // Release [range_start, range_end): clear only entries this owner
    // still holds (a later claimant's entries are left alone).
    void ReleaseReverseMap(uint16_t owner, uint16_t range_start, uint16_t range_end)
    {
        for (uint16_t a = range_start; a != range_end; a++)
        {
            if (reverseMap_[a] == owner)
                reverseMap_[a] = NO_BLOCK;
        }
    }

    bool ValidateDecodedBlock(uint16_t start_pc, int num_insns,
                              const DecodedInst* insns, uint16_t decode_end_pc,
                              uint16_t taken_mask = 0) const
    {
        if (num_insns <= 0 || num_insns > MAX_BLOCK_INSNS)
            return false;

        uint16_t pc = start_pc;
        for (int i = 0; i < num_insns; i++)
        {
            if (insns[i].handler == nullptr || insns[i].length == 0)
                return false;
            pc = (uint16_t)(pc + insns[i].length);
            if ((taken_mask >> i) & 1u)
                pc = (uint16_t)(pc + (int8_t)(insns[i].operand & 0xFF));
        }

        return pc == decode_end_pc;
    }

    // Walk a (possibly non-contiguous) block's contiguous address
    // ranges, calling fn(start, end) for each. Taken-direction traces
    // jump at every taken_mask bit; everything between two jumps is
    // one range. Used for reverse-map and page-bitmap maintenance.
    template <typename Fn>
    static void ForEachBlockRange(uint16_t start_pc, int num_insns,
                                  const DecodedInst* insns,
                                  uint16_t taken_mask, Fn fn)
    {
        uint16_t run_start = start_pc;
        uint16_t pc = start_pc;
        for (int i = 0; i < num_insns; i++)
        {
            pc = (uint16_t)(pc + insns[i].length);
            if (((taken_mask >> i) & 1u) && i < num_insns - 1)
            {
                fn(run_start, pc);
                pc = (uint16_t)(pc + (int8_t)(insns[i].operand & 0xFF));
                run_start = pc;
            }
        }
        fn(run_start, pc);
    }

    void FinishRecord(uint16_t end_pc, int cycle_counter)
    {
        recording_ = false;
        int total_cycles = cycle_counter - rec_cycle_start_;
        (void)end_pc;

        // Only cache if we got useful data
        if (rec_insn_count_ > 0 && total_cycles > 0 && total_cycles < 256)
        {
            // Pre-decode instructions for the block execution path,
            // following the recorded taken-guard directions.
            DecodedInst decoded[MAX_BLOCK_INSNS];
            uint16_t decode_end_pc = DecodeBlock(rec_start_pc_, rec_insn_count_,
                                                 decoded, rec_taken_mask_);
            if (!ValidateDecodedBlock(rec_start_pc_, rec_insn_count_, decoded,
                                      decode_end_pc, rec_taken_mask_))
            {
                stats_.rejected_blocks++;
                return;
            }

            // Temporary diagnostic (VCC_LOG_TRACE): dump every cached
            // taken-trace block and flag mask bits on non-branch shapes.
            static const bool log_trace = getenv("VCC_LOG_TRACE") != nullptr;
            if (log_trace && rec_taken_mask_ != 0)
            {
                fprintf(stderr, "[TRACE] pc=%04X n=%d mask=%04X end=%04X |",
                        rec_start_pc_, rec_insn_count_, rec_taken_mask_,
                        decode_end_pc);
                for (int i = 0; i < rec_insn_count_; ++i)
                {
                    fprintf(stderr, " %02X/%d%s",
                            (unsigned)(decoded[i].operand & 0xFF),
                            decoded[i].length,
                            ((rec_taken_mask_ >> i) & 1u) ? "T" : "");
                    if (((rec_taken_mask_ >> i) & 1u) && decoded[i].length != 2)
                        fprintf(stderr, "(BAD)");
                }
                fprintf(stderr, "\n");
            }

            CachedBlock& old = blocks_[rec_start_pc_ & CACHE_MASK];

            // Incumbent survival: a thunk's validity depends only on
            // the slot's start_pc, insns[], and taken_mask (it bakes
            // handler pointers, operands, guard directions, and the PC
            // chain derived from them). Bulk invalidations - every MMU
            // remap under OS-9 - leave the slot bytes intact. If the
            // incumbent block still matches MEMORY along its own
            // recorded path, it stays - its guards make it correct for
            // every branch outcome, so a re-record that merely observed
            // different guard directions must not flush the thunk and
            // hotness. Only a genuine code change replaces the block.
            if (old.native_entry != nullptr &&
                old.start_pc == rec_start_pc_)
            {
                // Fast path: same shape as the new recording - compare
                // against the decode we already have instead of
                // re-decoding (the overwhelmingly common re-record).
                bool matches;
                if (old.num_insns == (uint8_t)rec_insn_count_ &&
                    old.taken_mask == rec_taken_mask_)
                {
                    matches = old.end_pc == decode_end_pc &&
                              memcmp(old.insns, decoded,
                                     (size_t)rec_insn_count_ * sizeof(DecodedInst)) == 0;
                }
                else
                {
                    DecodedInst cur[MAX_BLOCK_INSNS];
                    const uint16_t cur_end =
                        DecodeBlock(rec_start_pc_, old.num_insns, cur, old.taken_mask);
                    matches = cur_end == old.end_pc &&
                              memcmp(old.insns, cur,
                                     (size_t)old.num_insns * sizeof(DecodedInst)) == 0;
                }
                if (matches)
                {
                    old.generation = generation_;
                    ForEachBlockRange(old.start_pc, old.num_insns, old.insns,
                                      old.taken_mask,
                                      [this, &old](uint16_t s, uint16_t e) {
                                          ClaimReverseMap(old.start_pc, s, e);
                                          MarkPageBits(s, e);
                                      });
                    return;
                }
            }

            // If replacing a valid block, clear its reverse map first.
            if (old.generation == generation_)
                ForEachBlockRange(old.start_pc, old.num_insns, old.insns,
                                  old.taken_mask,
                                  [this, &old](uint16_t s, uint16_t e) {
                                      ReleaseReverseMap(old.start_pc, s, e);
                                  });

            old.start_pc = rec_start_pc_;
            old.end_pc = decode_end_pc;
            old.num_insns = (uint8_t)rec_insn_count_;
            old.total_cycles = (uint8_t)total_cycles;
            old.generation = generation_;
            old.taken_mask = rec_taken_mask_;
            // New contents: any thunk in this slot bakes the OLD
            // block's instructions - drop it so the dispatch loop uses
            // the interpreter path (and the block earns a fresh thunk
            // once hot again).
            old.native_entry = nullptr;
            old.exec_count = 0;
            old.pure_thunk = 0;
            memcpy(old.insns, decoded, sizeof(decoded));

            ForEachBlockRange(old.start_pc, old.num_insns, old.insns,
                              old.taken_mask,
                              [this, &old](uint16_t s, uint16_t e) {
                                  ClaimReverseMap(old.start_pc, s, e);
                                  MarkPageBits(s, e);
                              });
            stats_.blocks_recorded++;
        }
    }

    // Mark page bitmap bits for one contiguous range. Ranges are
    // bounded by MAX_BLOCK_INSNS instruction lengths, so a range spans
    // one or two 256-byte pages; marking both endpoints avoids the old
    // uint16_t wraparound loop bug for ranges straddling 0xFFFF.
    void MarkPageBits(uint16_t start, uint16_t end)
    {
        SetPageBit(start);
        uint16_t last_byte = (uint16_t)(end - 1);
        if ((last_byte & 0xFF00) != (start & 0xFF00))
            SetPageBit(last_byte);
    }
};
