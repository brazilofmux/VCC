#ifndef EMIT_X64_H
#define EMIT_X64_H

#include <stdint.h>
#include <string.h>

/*
 * Lightweight x86-64 code emitter for JIT compilation.
 *
 * Adapted from ~/riscv/dbt/emit_x64.h (same author, same single-header
 * static-inline style as emit_a64.h). The RISC-V donor pinned RBX/R12
 * and addressed guest memory through SIB forms; this version provides
 * general [base + disp32] operands instead, because the VCC backend
 * pins R13 at the Hd6309State base and does byte-granular RMW directly
 * on state memory — the idiom the retired x86-32 backend proved out
 * (inc/neg/shl on the accumulator byte, setcc straight into cc[]).
 *
 * Host register convention used by BlockJitX64.cpp:
 *   R13 = &cpu_state (Hd6309State base)   (callee-saved)
 *   R14 = &slot.insns[0]                  (callee-saved)
 *   EBX = CycleCounter                    (callee-saved, registerized)
 *   R12D = CycleFor                       (callee-saved, chain budget)
 *   R15 = block-cache slot base           (callee-saved, chain stub)
 *   RAX, RCX, RDX, R8-R11 = scratch       (caller-saved on both ABIs)
 *
 * RSI/RDI are deliberately absent from the scratch set: they are
 * callee-saved on Win64 and argument registers on SysV, so the only
 * safe uniform use is "SysV argument register", which the ARG macros
 * in BlockJitX64.cpp handle.
 */

/* Code buffer cursor — same shape as emit_t in emit_a64.h. */
typedef struct {
    uint8_t *buf;
    uint32_t offset;
    uint32_t capacity;
} emit_t;

static inline void emit_byte(emit_t *e, uint8_t b) {
    if (e->offset < e->capacity)
        e->buf[e->offset] = b;
    e->offset++;
}

static inline void emit_bytes(emit_t *e, const void *data, int len) {
    if (e->offset + (uint32_t)len <= e->capacity)
        memcpy(e->buf + e->offset, data, len);
    e->offset += (uint32_t)len;
}

static inline void emit_u16(emit_t *e, uint16_t val) {
    emit_bytes(e, &val, 2);
}

static inline void emit_u32(emit_t *e, uint32_t val) {
    emit_bytes(e, &val, 4);
}

static inline void emit_u64(emit_t *e, uint64_t val) {
    emit_bytes(e, &val, 8);
}

static inline uint32_t emit_pos(emit_t *e) {
    return e->offset;
}

/* Patch a 32-bit relative displacement whose 4-byte field sits at
 * patch_offset; the displacement is measured from the END of the field
 * (i.e., the next instruction), per x86 rel32 semantics. */
static inline void emit_patch_rel32(emit_t *e, uint32_t patch_offset, uint32_t target_offset) {
    int32_t disp = (int32_t)(target_offset - (patch_offset + 4));
    memcpy(e->buf + patch_offset, &disp, 4);
}

/* x86-64 register encoding */
#define X64_RAX 0
#define X64_RCX 1
#define X64_RDX 2
#define X64_RBX 3
#define X64_RSP 4
#define X64_RBP 5
#define X64_RSI 6
#define X64_RDI 7
#define X64_R8  8
#define X64_R9  9
#define X64_R10 10
#define X64_R11 11
#define X64_R12 12
#define X64_R13 13
#define X64_R14 14
#define X64_R15 15

/* REX prefix helpers */
static inline uint8_t x64_rex(int w, int r, int x, int b) {
    return (uint8_t)(0x40 | (w << 3) | (r << 2) | (x << 1) | b);
}

static inline int reg_hi(int r) { return (r >> 3) & 1; }
static inline int reg_lo(int r) { return r & 7; }

/* ModR/M byte */
static inline uint8_t x64_modrm(int mod, int reg, int rm) {
    return (uint8_t)((mod << 6) | ((reg & 7) << 3) | (rm & 7));
}

/* Emit REX only when required. For 8-bit register operands, pass
 * force8=1 with any register >= 4 so SPL/BPL/SIL/DIL encode instead of
 * AH/CH/DH/BH — callers below handle that where it matters. */
static inline void emit_rex_opt(emit_t *e, int w, int r, int x, int b) {
    if (w || r || x || b)
        emit_byte(e, x64_rex(w, r, x, b));
}

/* ---- memory operand: [base + disp] ----
 * Handles the two ModRM irregularities:
 *   base lo3 == 4 (RSP/R12): needs a SIB byte (index=none)
 *   base lo3 == 5 (RBP/R13): mod=00 means disp32-absolute/RIP, so a
 *     zero displacement must still use the disp8 form
 */
static inline void emit_mem(emit_t *e, int reg_field, int base, int32_t disp) {
    const int lo = reg_lo(base);
    const int need_sib = (lo == 4);
    const int force_disp = (lo == 5);
    int mod;
    if (disp == 0 && !force_disp) mod = 0x00;
    else if (disp >= -128 && disp <= 127) mod = 0x01;
    else mod = 0x02;
    emit_byte(e, x64_modrm(mod, reg_field, need_sib ? 4 : lo));
    if (need_sib)
        emit_byte(e, (uint8_t)((4 << 3) | lo));   /* no index */
    if (mod == 0x01) emit_byte(e, (uint8_t)(int8_t)disp);
    else if (mod == 0x02) emit_u32(e, (uint32_t)disp);
}

/* ---- memory operand with SIB: [base + index*scale + disp] ----
 * index must not be RSP (lo3 == 4 means "no index"); base RBP/R13
 * forces the disp8 form as in emit_mem. scale_log is 0..3. */
static inline void emit_mem_sib(emit_t *e, int reg_field, int base, int index,
                                int scale_log, int32_t disp) {
    const int lo = reg_lo(base);
    const int force_disp = (lo == 5);
    int mod;
    if (disp == 0 && !force_disp) mod = 0x00;
    else if (disp >= -128 && disp <= 127) mod = 0x01;
    else mod = 0x02;
    emit_byte(e, x64_modrm(mod, reg_field, 4));
    emit_byte(e, (uint8_t)((scale_log << 6) | (reg_lo(index) << 3) | lo));
    if (mod == 0x01) emit_byte(e, (uint8_t)(int8_t)disp);
    else if (mod == 0x02) emit_u32(e, (uint32_t)disp);
}

/* mov r64, [base + index*(2^scale_log) + disp] */
static inline void emit_load64_sib(emit_t *e, int dst, int base, int index,
                                   int scale_log, int32_t disp) {
    emit_byte(e, x64_rex(1, reg_hi(dst), reg_hi(index), reg_hi(base)));
    emit_byte(e, 0x8B);
    emit_mem_sib(e, dst, base, index, scale_log, disp);
}

/* movzx r32, byte [base + index + disp] */
static inline void emit_load8u_sib(emit_t *e, int dst, int base, int index,
                                   int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(dst), reg_hi(index), reg_hi(base));
    emit_byte(e, 0x0F); emit_byte(e, 0xB6);
    emit_mem_sib(e, dst, base, index, 0, disp);
}

/* mov byte [base + index + disp], r8 — REX forced for src >= 4 */
static inline void emit_store8_sib(emit_t *e, int src, int base, int index,
                                   int32_t disp) {
    if (src >= 4 || reg_hi(index) || reg_hi(base))
        emit_byte(e, x64_rex(0, reg_hi(src), reg_hi(index), reg_hi(base)));
    emit_byte(e, 0x88);
    emit_mem_sib(e, src, base, index, 0, disp);
}

/* bt r32, r32 — CF = bit (bit_reg mod 32) of val_reg */
static inline void emit_bt_rr(emit_t *e, int val, int bit) {
    emit_rex_opt(e, 0, reg_hi(bit), 0, reg_hi(val));
    emit_byte(e, 0x0F); emit_byte(e, 0xA3);
    emit_byte(e, x64_modrm(0x03, bit, val));
}

/* ---- loads (zero/sign extend into r32) ---- */

/* movzx r32, byte [base + disp] */
static inline void emit_load8u(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(dst), 0, reg_hi(base));
    emit_byte(e, 0x0F); emit_byte(e, 0xB6);
    emit_mem(e, dst, base, disp);
}

/* movsx r32, byte [base + disp] */
static inline void emit_load8s(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(dst), 0, reg_hi(base));
    emit_byte(e, 0x0F); emit_byte(e, 0xBE);
    emit_mem(e, dst, base, disp);
}

/* movzx r32, word [base + disp] */
static inline void emit_load16u(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(dst), 0, reg_hi(base));
    emit_byte(e, 0x0F); emit_byte(e, 0xB7);
    emit_mem(e, dst, base, disp);
}

/* mov r32, [base + disp] */
static inline void emit_load32(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(dst), 0, reg_hi(base));
    emit_byte(e, 0x8B);
    emit_mem(e, dst, base, disp);
}

/* mov r64, [base + disp] */
static inline void emit_load64(emit_t *e, int dst, int base, int32_t disp) {
    emit_byte(e, x64_rex(1, reg_hi(dst), 0, reg_hi(base)));
    emit_byte(e, 0x8B);
    emit_mem(e, dst, base, disp);
}

/* ---- stores ---- */

/* mov byte [base + disp], r8 — REX forced for src >= 4 (SPL..DIL trap) */
static inline void emit_store8(emit_t *e, int src, int base, int32_t disp) {
    if (src >= 4 || reg_hi(base))
        emit_byte(e, x64_rex(0, reg_hi(src), 0, reg_hi(base)));
    emit_byte(e, 0x88);
    emit_mem(e, src, base, disp);
}

/* mov word [base + disp], r16 */
static inline void emit_store16(emit_t *e, int src, int base, int32_t disp) {
    emit_byte(e, 0x66);
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(base));
    emit_byte(e, 0x89);
    emit_mem(e, src, base, disp);
}

/* mov [base + disp], r32 */
static inline void emit_store32(emit_t *e, int src, int base, int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(base));
    emit_byte(e, 0x89);
    emit_mem(e, src, base, disp);
}

/* mov [base + disp], r64 */
static inline void emit_store64(emit_t *e, int src, int base, int32_t disp) {
    emit_byte(e, x64_rex(1, reg_hi(src), 0, reg_hi(base)));
    emit_byte(e, 0x89);
    emit_mem(e, src, base, disp);
}

/* ---- immediates to memory ---- */

/* mov byte [base + disp], imm8 */
static inline void emit_mov_mem8_imm8(emit_t *e, int base, int32_t disp, uint8_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0xC6);
    emit_mem(e, 0, base, disp);
    emit_byte(e, imm);
}

/* mov word [base + disp], imm16 */
static inline void emit_mov_mem16_imm16(emit_t *e, int base, int32_t disp, uint16_t imm) {
    emit_byte(e, 0x66);
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0xC7);
    emit_mem(e, 0, base, disp);
    emit_u16(e, imm);
}

/* ---- byte RMW on memory (sets host flags — the whole point) ---- */

/* Group-1 ALU op numbers for the /r field of opcodes 80 (mem8,imm8). */
#define X64_ALU_ADD 0
#define X64_ALU_OR  1
#define X64_ALU_AND 4
#define X64_ALU_SUB 5
#define X64_ALU_XOR 6
#define X64_ALU_CMP 7

/* op byte [base + disp], imm8 — add/or/and/sub/xor/cmp */
static inline void emit_alu_mem8_imm8(emit_t *e, int op, int base, int32_t disp, uint8_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0x80);
    emit_mem(e, op, base, disp);
    emit_byte(e, imm);
}

/* op r8, imm8 (register-direct form of group 1) */
static inline void emit_alu_r8_imm8(emit_t *e, int op, int reg, uint8_t imm) {
    if (reg >= 4)
        emit_byte(e, x64_rex(0, 0, 0, reg_hi(reg)));
    emit_byte(e, 0x80);
    emit_byte(e, x64_modrm(0x03, op, reg));
    emit_byte(e, imm);
}

/* test byte [base + disp], imm8 */
static inline void emit_test_mem8_imm8(emit_t *e, int base, int32_t disp, uint8_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0xF6);
    emit_mem(e, 0, base, disp);
    emit_byte(e, imm);
}

/* inc byte [base + disp] — CF preserved, OF on 0x7F->0x80 */
static inline void emit_inc_mem8(emit_t *e, int base, int32_t disp) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0xFE);
    emit_mem(e, 0, base, disp);
}

/* dec byte [base + disp] — CF preserved, OF on 0x80->0x7F */
static inline void emit_dec_mem8(emit_t *e, int base, int32_t disp) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0xFE);
    emit_mem(e, 1, base, disp);
}

/* neg byte [base + disp] — CF = (input != 0), OF = (input == 0x80) */
static inline void emit_neg_mem8(emit_t *e, int base, int32_t disp) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0xF6);
    emit_mem(e, 3, base, disp);
}

/* Shift-by-1 op numbers for the /r field of opcode D0 (mem8, 1). */
#define X64_SHIFT_SHL 4
#define X64_SHIFT_SHR 5
#define X64_SHIFT_SAR 7

/* shl/shr/sar byte [base + disp], 1 */
static inline void emit_shift_mem8_1(emit_t *e, int op, int base, int32_t disp) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0xD0);
    emit_mem(e, op, base, disp);
}

/* add word [base + disp], r16 (ABX folds B into X this way) */
static inline void emit_add_mem16_r16(emit_t *e, int base, int32_t disp, int src) {
    emit_byte(e, 0x66);
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(base));
    emit_byte(e, 0x01);
    emit_mem(e, src, base, disp);
}

/* cmp word [base + disp], r16 */
static inline void emit_cmp_mem16_r16(emit_t *e, int base, int32_t disp, int src) {
    emit_byte(e, 0x66);
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(base));
    emit_byte(e, 0x39);
    emit_mem(e, src, base, disp);
}

/* cmp r32, [base + disp] */
static inline void emit_cmp_r32_mem32(emit_t *e, int reg, int base, int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(reg), 0, reg_hi(base));
    emit_byte(e, 0x3B);
    emit_mem(e, reg, base, disp);
}

/* add qword [base + disp], imm8 (stats counters) */
static inline void emit_add_mem64_imm8(emit_t *e, int base, int32_t disp, int8_t imm) {
    emit_byte(e, x64_rex(1, 0, 0, reg_hi(base)));
    emit_byte(e, 0x83);
    emit_mem(e, 0, base, disp);
    emit_byte(e, (uint8_t)imm);
}

/* ---- setcc ---- */

/* Condition nibbles (opcode = 0F 9x for setcc, 0F 8x for jcc, 0F 4x for cmovcc) */
#define X64_CC_O   0x0
#define X64_CC_NO  0x1
#define X64_CC_B   0x2   /* CF=1: unsigned <, carry/borrow */
#define X64_CC_AE  0x3   /* CF=0 */
#define X64_CC_E   0x4   /* ZF=1 */
#define X64_CC_NE  0x5   /* ZF=0 */
#define X64_CC_BE  0x6
#define X64_CC_A   0x7
#define X64_CC_S   0x8   /* SF=1 */
#define X64_CC_NS  0x9
#define X64_CC_L   0xC   /* signed < */
#define X64_CC_GE  0xD
#define X64_CC_LE  0xE
#define X64_CC_G   0xF

/* setcc byte [base + disp] */
static inline void emit_setcc_mem(emit_t *e, int cc, int base, int32_t disp) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(base));
    emit_byte(e, 0x0F);
    emit_byte(e, (uint8_t)(0x90 + cc));
    emit_mem(e, 0, base, disp);
}

/* ---- register-register / register-immediate ALU (32-bit) ---- */

/* mov r32, r32 */
static inline void emit_mov_rr(emit_t *e, int dst, int src) {
    if (dst == src) return;
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(dst));
    emit_byte(e, 0x89);
    emit_byte(e, x64_modrm(0x03, src, dst));
}

/* mov r32, imm32 */
static inline void emit_mov_r32_imm32(emit_t *e, int reg, uint32_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    emit_byte(e, (uint8_t)(0xB8 + reg_lo(reg)));
    emit_u32(e, imm);
}

/* mov r64, imm64 */
static inline void emit_mov_r64_imm64(emit_t *e, int reg, uint64_t imm) {
    emit_byte(e, x64_rex(1, 0, 0, reg_hi(reg)));
    emit_byte(e, (uint8_t)(0xB8 + reg_lo(reg)));
    emit_u64(e, imm);
}

/* add r32, r32 */
static inline void emit_add_rr(emit_t *e, int dst, int src) {
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(dst));
    emit_byte(e, 0x01);
    emit_byte(e, x64_modrm(0x03, src, dst));
}

/* sub r32, r32 */
static inline void emit_sub_rr(emit_t *e, int dst, int src) {
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(dst));
    emit_byte(e, 0x29);
    emit_byte(e, x64_modrm(0x03, src, dst));
}

/* or r32, r32 */
static inline void emit_or_rr(emit_t *e, int dst, int src) {
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(dst));
    emit_byte(e, 0x09);
    emit_byte(e, x64_modrm(0x03, src, dst));
}

/* xor r32, r32 */
static inline void emit_xor_rr(emit_t *e, int dst, int src) {
    emit_rex_opt(e, 0, reg_hi(src), 0, reg_hi(dst));
    emit_byte(e, 0x31);
    emit_byte(e, x64_modrm(0x03, src, dst));
}

/* mov r64, r64 */
static inline void emit_mov_r64_r64(emit_t *e, int dst, int src) {
    if (dst == src) return;
    emit_byte(e, x64_rex(1, reg_hi(src), 0, reg_hi(dst)));
    emit_byte(e, 0x89);
    emit_byte(e, x64_modrm(0x03, src, dst));
}

/* add r64, r64 */
static inline void emit_add_r64_r64(emit_t *e, int dst, int src) {
    emit_byte(e, x64_rex(1, reg_hi(src), 0, reg_hi(dst)));
    emit_byte(e, 0x01);
    emit_byte(e, x64_modrm(0x03, src, dst));
}

/* add r32, imm (imm8 form when it fits; no-op for imm == 0) */
static inline void emit_add_r_imm(emit_t *e, int reg, int32_t imm) {
    if (imm == 0) return;
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (imm >= -128 && imm <= 127) {
        emit_byte(e, 0x83);
        emit_byte(e, x64_modrm(0x03, 0, reg));
        emit_byte(e, (uint8_t)(int8_t)imm);
    } else {
        emit_byte(e, 0x81);
        emit_byte(e, x64_modrm(0x03, 0, reg));
        emit_u32(e, (uint32_t)imm);
    }
}

/* and r32, imm */
static inline void emit_and_r_imm(emit_t *e, int reg, int32_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (imm >= -128 && imm <= 127) {
        emit_byte(e, 0x83);
        emit_byte(e, x64_modrm(0x03, 4, reg));
        emit_byte(e, (uint8_t)(int8_t)imm);
    } else {
        emit_byte(e, 0x81);
        emit_byte(e, x64_modrm(0x03, 4, reg));
        emit_u32(e, (uint32_t)imm);
    }
}

/* or r32, imm */
static inline void emit_or_r_imm(emit_t *e, int reg, int32_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (imm >= -128 && imm <= 127) {
        emit_byte(e, 0x83);
        emit_byte(e, x64_modrm(0x03, 1, reg));
        emit_byte(e, (uint8_t)(int8_t)imm);
    } else {
        emit_byte(e, 0x81);
        emit_byte(e, x64_modrm(0x03, 1, reg));
        emit_u32(e, (uint32_t)imm);
    }
}

/* xor r32, imm */
static inline void emit_xor_r_imm(emit_t *e, int reg, int32_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (imm >= -128 && imm <= 127) {
        emit_byte(e, 0x83);
        emit_byte(e, x64_modrm(0x03, 6, reg));
        emit_byte(e, (uint8_t)(int8_t)imm);
    } else {
        emit_byte(e, 0x81);
        emit_byte(e, x64_modrm(0x03, 6, reg));
        emit_u32(e, (uint32_t)imm);
    }
}

/* cmp r32, r32 */
static inline void emit_cmp_rr(emit_t *e, int r1, int r2) {
    emit_rex_opt(e, 0, reg_hi(r2), 0, reg_hi(r1));
    emit_byte(e, 0x39);
    emit_byte(e, x64_modrm(0x03, r2, r1));
}

/* cmp r32, imm */
static inline void emit_cmp_r_imm(emit_t *e, int reg, int32_t imm) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (imm >= -128 && imm <= 127) {
        emit_byte(e, 0x83);
        emit_byte(e, x64_modrm(0x03, 7, reg));
        emit_byte(e, (uint8_t)(int8_t)imm);
    } else {
        emit_byte(e, 0x81);
        emit_byte(e, x64_modrm(0x03, 7, reg));
        emit_u32(e, (uint32_t)imm);
    }
}

/* test r32, r32 */
static inline void emit_test_rr(emit_t *e, int r1, int r2) {
    emit_rex_opt(e, 0, reg_hi(r2), 0, reg_hi(r1));
    emit_byte(e, 0x85);
    emit_byte(e, x64_modrm(0x03, r2, r1));
}

/* test r64, r64 */
static inline void emit_test_r64_r64(emit_t *e, int r1, int r2) {
    emit_byte(e, x64_rex(1, reg_hi(r2), 0, reg_hi(r1)));
    emit_byte(e, 0x85);
    emit_byte(e, x64_modrm(0x03, r2, r1));
}

/* test r8, r8 — REX forced for regs >= 4 (SPL..DIL trap) */
static inline void emit_test_r8_r8(emit_t *e, int r1, int r2) {
    if (r1 >= 4 || r2 >= 4)
        emit_byte(e, x64_rex(0, reg_hi(r2), 0, reg_hi(r1)));
    emit_byte(e, 0x84);
    emit_byte(e, x64_modrm(0x03, r2, r1));
}

/* test r16, r16 — ZF/SF from the low 16 bits (SF = bit 15) */
static inline void emit_test_r16_r16(emit_t *e, int r1, int r2) {
    emit_byte(e, 0x66);
    emit_rex_opt(e, 0, reg_hi(r2), 0, reg_hi(r1));
    emit_byte(e, 0x85);
    emit_byte(e, x64_modrm(0x03, r2, r1));
}

/* shl/shr/sar r32, imm8 */
static inline void emit_shl_r_imm(emit_t *e, int reg, uint8_t amt) {
    if (amt == 0) return;
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (amt == 1) { emit_byte(e, 0xD1); emit_byte(e, x64_modrm(0x03, 4, reg)); }
    else { emit_byte(e, 0xC1); emit_byte(e, x64_modrm(0x03, 4, reg)); emit_byte(e, amt); }
}

static inline void emit_shr_r_imm(emit_t *e, int reg, uint8_t amt) {
    if (amt == 0) return;
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (amt == 1) { emit_byte(e, 0xD1); emit_byte(e, x64_modrm(0x03, 5, reg)); }
    else { emit_byte(e, 0xC1); emit_byte(e, x64_modrm(0x03, 5, reg)); emit_byte(e, amt); }
}

static inline void emit_sar_r_imm(emit_t *e, int reg, uint8_t amt) {
    if (amt == 0) return;
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    if (amt == 1) { emit_byte(e, 0xD1); emit_byte(e, x64_modrm(0x03, 7, reg)); }
    else { emit_byte(e, 0xC1); emit_byte(e, x64_modrm(0x03, 7, reg)); emit_byte(e, amt); }
}

/* shl r64, imm8 */
static inline void emit_shl_r64_imm(emit_t *e, int reg, uint8_t amt) {
    if (amt == 0) return;
    emit_byte(e, x64_rex(1, 0, 0, reg_hi(reg)));
    if (amt == 1) { emit_byte(e, 0xD1); emit_byte(e, x64_modrm(0x03, 4, reg)); }
    else { emit_byte(e, 0xC1); emit_byte(e, x64_modrm(0x03, 4, reg)); emit_byte(e, amt); }
}

/* movzx r32, r8 — always REX so regs 4-7 map to SPL..DIL, not AH..BH */
static inline void emit_movzx_r32_r8(emit_t *e, int dst, int src) {
    emit_byte(e, x64_rex(0, reg_hi(dst), 0, reg_hi(src)));
    emit_byte(e, 0x0F); emit_byte(e, 0xB6);
    emit_byte(e, x64_modrm(0x03, dst, src));
}

/* movzx r32, r16 */
static inline void emit_movzx_r32_r16(emit_t *e, int dst, int src) {
    emit_rex_opt(e, 0, reg_hi(dst), 0, reg_hi(src));
    emit_byte(e, 0x0F); emit_byte(e, 0xB7);
    emit_byte(e, x64_modrm(0x03, dst, src));
}

/* movsx r32, r8 — always REX, same rationale as movzx */
static inline void emit_movsx_r32_r8(emit_t *e, int dst, int src) {
    emit_byte(e, x64_rex(0, reg_hi(dst), 0, reg_hi(src)));
    emit_byte(e, 0x0F); emit_byte(e, 0xBE);
    emit_byte(e, x64_modrm(0x03, dst, src));
}

/* cmovcc r32, r32 */
static inline void emit_cmovcc(emit_t *e, int cc, int dst, int src) {
    emit_rex_opt(e, 0, reg_hi(dst), 0, reg_hi(src));
    emit_byte(e, 0x0F);
    emit_byte(e, (uint8_t)(0x40 + cc));
    emit_byte(e, x64_modrm(0x03, dst, src));
}

/* lea r64, [base + disp] */
static inline void emit_lea64(emit_t *e, int dst, int base, int32_t disp) {
    emit_byte(e, x64_rex(1, reg_hi(dst), 0, reg_hi(base)));
    emit_byte(e, 0x8D);
    emit_mem(e, dst, base, disp);
}

/* lea r32, [base + disp] (32-bit result; base is still a 64-bit reg) */
static inline void emit_lea32(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex_opt(e, 0, reg_hi(dst), 0, reg_hi(base));
    emit_byte(e, 0x8D);
    emit_mem(e, dst, base, disp);
}

/* ---- control flow ---- */

/* call r64 */
static inline void emit_call_r(emit_t *e, int reg) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    emit_byte(e, 0xFF);
    emit_byte(e, x64_modrm(0x03, 2, reg));
}

/* jmp r64 */
static inline void emit_jmp_r(emit_t *e, int reg) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    emit_byte(e, 0xFF);
    emit_byte(e, x64_modrm(0x03, 4, reg));
}

/* jmp rel32 — disp measured from the end of this instruction */
static inline void emit_jmp_rel32(emit_t *e, int32_t disp) {
    emit_byte(e, 0xE9);
    emit_u32(e, (uint32_t)disp);
}

/* jcc rel32 — disp measured from the end of this instruction; emit 0
 * and patch via emit_patch_rel32 when the target isn't known yet (the
 * 4-byte field sits at emit_pos()-4 right after this call) */
static inline void emit_jcc_rel32(emit_t *e, int cc, int32_t disp) {
    emit_byte(e, 0x0F);
    emit_byte(e, (uint8_t)(0x80 + cc));
    emit_u32(e, (uint32_t)disp);
}

/* ret */
static inline void emit_x64_ret(emit_t *e) {
    emit_byte(e, 0xC3);
}

/* push/pop r64 */
static inline void emit_push(emit_t *e, int reg) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    emit_byte(e, (uint8_t)(0x50 + reg_lo(reg)));
}

static inline void emit_pop(emit_t *e, int reg) {
    emit_rex_opt(e, 0, 0, 0, reg_hi(reg));
    emit_byte(e, (uint8_t)(0x58 + reg_lo(reg)));
}

/* sub/add rsp, imm8 */
static inline void emit_sub_rsp_imm8(emit_t *e, int8_t imm) {
    emit_byte(e, x64_rex(1, 0, 0, 0));
    emit_byte(e, 0x83);
    emit_byte(e, x64_modrm(0x03, 5, X64_RSP));
    emit_byte(e, (uint8_t)imm);
}

static inline void emit_add_rsp_imm8(emit_t *e, int8_t imm) {
    emit_byte(e, x64_rex(1, 0, 0, 0));
    emit_byte(e, 0x83);
    emit_byte(e, x64_modrm(0x03, 0, X64_RSP));
    emit_byte(e, (uint8_t)imm);
}

#endif /* EMIT_X64_H */
