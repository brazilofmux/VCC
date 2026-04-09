#pragma once
// MC6809 threaded interpreter handler functions.
// Each handler is a self-contained function that reads its own operands
// from pc.Reg, executes the operation, updates flags and CycleCounter.
// These access CPU state via the existing static globals in mc6809.cpp.
//
// The handler table maps opcode byte -> function pointer.
// Page 2 (0x10) and Page 3 (0x11) have their own tables.

typedef void (*OpcodeHandler)();

// ============================================================
// Helper: effective address calculation for each addressing mode
// ============================================================

// Direct page: EA = DP | next_byte
#define EA_DIRECT() (dp.Reg | MemRead8(pc.Reg++))

// Extended: EA = next_word
#define EA_EXTENDED() (MemRead16(pc.Reg)); pc.Reg += 2

// Indexed: EA = CalculateEA(postbyte)
#define EA_INDEXED() CalculateEA(MemRead8(pc.Reg++))

// ============================================================
// Read-modify-write memory operations (NEG, COM, LSR, etc.)
// These share the same pattern: read byte at EA, modify, write back.
// Four addressing modes: Direct, Indexed, Extended + Inherent A/B.
// ============================================================

// --- NEG: negate byte ---
#define BODY_NEG(val) \
    temp8 = 0 - (val); \
    cc[C] = temp8 > 0; \
    cc[V] = ((val) == 0x80); \
    cc[N] = NTEST8(temp8); \
    cc[Z] = ZTEST(temp8);

static void op_NEG_D() {
    temp16 = EA_DIRECT();
    postbyte = MemRead8(temp16);
    BODY_NEG(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_NEG_X() {
    temp16 = EA_INDEXED();
    postbyte = MemRead8(temp16);
    BODY_NEG(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_NEG_E() {
    temp16 = EA_EXTENDED();
    postbyte = MemRead8(temp16);
    BODY_NEG(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_NEGA() {
    BODY_NEG(A_REG);
    A_REG = temp8;
    CycleCounter += 2;
}
static void op_NEGB() {
    BODY_NEG(B_REG);
    B_REG = temp8;
    CycleCounter += 2;
}

// --- COM: complement byte ---
#define BODY_COM(val) \
    temp8 = 0xFF - (val); \
    cc[Z] = ZTEST(temp8); \
    cc[N] = NTEST8(temp8); \
    cc[C] = true; \
    cc[V] = false;

static void op_COM_D() {
    temp16 = EA_DIRECT();
    postbyte = MemRead8(temp16);
    BODY_COM(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_COM_X() {
    temp16 = EA_INDEXED();
    postbyte = MemRead8(temp16);
    BODY_COM(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_COM_E() {
    temp16 = EA_EXTENDED();
    postbyte = MemRead8(temp16);
    BODY_COM(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_COMA() {
    BODY_COM(A_REG);
    A_REG = temp8;
    CycleCounter += 2;
}
static void op_COMB() {
    BODY_COM(B_REG);
    B_REG = temp8;
    CycleCounter += 2;
}

// --- LSR: logical shift right ---
#define BODY_LSR(val) \
    cc[C] = (val) & 1; \
    temp8 = (val) >> 1; \
    cc[Z] = ZTEST(temp8); \
    cc[N] = false;

static void op_LSR_D() {
    temp16 = EA_DIRECT();
    postbyte = MemRead8(temp16);
    BODY_LSR(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_LSR_X() {
    temp16 = EA_INDEXED();
    postbyte = MemRead8(temp16);
    BODY_LSR(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_LSR_E() {
    temp16 = EA_EXTENDED();
    postbyte = MemRead8(temp16);
    BODY_LSR(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_LSRA() {
    BODY_LSR(A_REG);
    A_REG = temp8;
    CycleCounter += 2;
}
static void op_LSRB() {
    BODY_LSR(B_REG);
    B_REG = temp8;
    CycleCounter += 2;
}

// --- ROR: rotate right through carry ---
static void op_ROR_D() {
    temp16 = EA_DIRECT();
    temp8 = MemRead8(temp16);
    postbyte = cc[C] << 7;
    cc[C] = temp8 & 1;
    temp8 = (temp8 >> 1) | postbyte;
    cc[Z] = ZTEST(temp8);
    cc[N] = NTEST8(temp8);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ROR_X() {
    temp16 = EA_INDEXED();
    temp8 = MemRead8(temp16);
    postbyte = cc[C] << 7;
    cc[C] = temp8 & 1;
    temp8 = (temp8 >> 1) | postbyte;
    cc[Z] = ZTEST(temp8);
    cc[N] = NTEST8(temp8);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ROR_E() {
    temp16 = EA_EXTENDED();
    temp8 = MemRead8(temp16);
    postbyte = cc[C] << 7;
    cc[C] = temp8 & 1;
    temp8 = (temp8 >> 1) | postbyte;
    cc[Z] = ZTEST(temp8);
    cc[N] = NTEST8(temp8);
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_RORA() {
    postbyte = cc[C] << 7;
    cc[C] = A_REG & 1;
    A_REG = (A_REG >> 1) | postbyte;
    cc[Z] = ZTEST(A_REG);
    cc[N] = NTEST8(A_REG);
    CycleCounter += 2;
}
static void op_RORB() {
    postbyte = cc[C] << 7;
    cc[C] = B_REG & 1;
    B_REG = (B_REG >> 1) | postbyte;
    cc[Z] = ZTEST(B_REG);
    cc[N] = NTEST8(B_REG);
    CycleCounter += 2;
}

// --- ASR: arithmetic shift right ---
#define BODY_ASR(val) \
    cc[C] = (val) & 1; \
    temp8 = ((val) & 0x80) | ((val) >> 1); \
    cc[Z] = ZTEST(temp8); \
    cc[N] = NTEST8(temp8);

static void op_ASR_D() {
    temp16 = EA_DIRECT();
    postbyte = MemRead8(temp16);
    BODY_ASR(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ASR_X() {
    temp16 = EA_INDEXED();
    postbyte = MemRead8(temp16);
    BODY_ASR(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ASR_E() {
    temp16 = EA_EXTENDED();
    postbyte = MemRead8(temp16);
    BODY_ASR(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_ASRA() {
    BODY_ASR(A_REG);
    A_REG = temp8;
    CycleCounter += 2;
}
static void op_ASRB() {
    BODY_ASR(B_REG);
    B_REG = temp8;
    CycleCounter += 2;
}

// --- ASL/LSL: arithmetic/logical shift left ---
#define BODY_ASL(val) \
    cc[C] = ((val) & 0x80) >> 7; \
    cc[V] = cc[C] ^ (((val) & 0x40) != 0); \
    temp8 = (val) << 1; \
    cc[N] = NTEST8(temp8); \
    cc[Z] = ZTEST(temp8);

static void op_ASL_D() {
    temp16 = EA_DIRECT();
    postbyte = MemRead8(temp16);
    BODY_ASL(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ASL_X() {
    temp16 = EA_INDEXED();
    postbyte = MemRead8(temp16);
    BODY_ASL(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ASL_E() {
    temp16 = EA_EXTENDED();
    postbyte = MemRead8(temp16);
    BODY_ASL(postbyte);
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_ASLA() {
    BODY_ASL(A_REG);
    A_REG = temp8;
    CycleCounter += 2;
}
static void op_ASLB() {
    BODY_ASL(B_REG);
    B_REG = temp8;
    CycleCounter += 2;
}

// --- ROL: rotate left through carry ---
static void op_ROL_D() {
    temp16 = EA_DIRECT();
    temp8 = MemRead8(temp16);
    postbyte = cc[C];
    cc[C] = (temp8 & 0x80) >> 7;
    cc[V] = cc[C] ^ ((temp8 & 0x40) != 0);
    temp8 = (temp8 << 1) | postbyte;
    cc[Z] = ZTEST(temp8);
    cc[N] = NTEST8(temp8);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ROL_X() {
    temp16 = EA_INDEXED();
    temp8 = MemRead8(temp16);
    postbyte = cc[C];
    cc[C] = (temp8 & 0x80) >> 7;
    cc[V] = cc[C] ^ ((temp8 & 0x40) != 0);
    temp8 = (temp8 << 1) | postbyte;
    cc[Z] = ZTEST(temp8);
    cc[N] = NTEST8(temp8);
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_ROL_E() {
    temp16 = EA_EXTENDED();
    temp8 = MemRead8(temp16);
    postbyte = cc[C];
    cc[C] = (temp8 & 0x80) >> 7;
    cc[V] = cc[C] ^ ((temp8 & 0x40) != 0);
    temp8 = (temp8 << 1) | postbyte;
    cc[Z] = ZTEST(temp8);
    cc[N] = NTEST8(temp8);
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_ROLA() {
    postbyte = cc[C];
    cc[C] = (A_REG & 0x80) >> 7;
    cc[V] = cc[C] ^ ((A_REG & 0x40) != 0);
    A_REG = (A_REG << 1) | postbyte;
    cc[Z] = ZTEST(A_REG);
    cc[N] = NTEST8(A_REG);
    CycleCounter += 2;
}
static void op_ROLB() {
    postbyte = cc[C];
    cc[C] = (B_REG & 0x80) >> 7;
    cc[V] = cc[C] ^ ((B_REG & 0x40) != 0);
    B_REG = (B_REG << 1) | postbyte;
    cc[Z] = ZTEST(B_REG);
    cc[N] = NTEST8(B_REG);
    CycleCounter += 2;
}

// --- DEC: decrement byte ---
#define BODY_DEC(val) \
    temp8 = (val) - 1; \
    cc[Z] = ZTEST(temp8); \
    cc[N] = NTEST8(temp8); \
    cc[V] = (temp8 == 0x7F);

static void op_DEC_D() {
    temp16 = EA_DIRECT();
    BODY_DEC(MemRead8(temp16));
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_DEC_X() {
    temp16 = EA_INDEXED();
    BODY_DEC(MemRead8(temp16));
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_DEC_E() {
    temp16 = EA_EXTENDED();
    BODY_DEC(MemRead8(temp16));
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_DECA() {
    BODY_DEC(A_REG);
    A_REG = temp8;
    CycleCounter += 2;
}
static void op_DECB() {
    BODY_DEC(B_REG);
    B_REG = temp8;
    CycleCounter += 2;
}

// --- INC: increment byte ---
#define BODY_INC(val) \
    temp8 = (val) + 1; \
    cc[Z] = ZTEST(temp8); \
    cc[V] = (temp8 == 0x80); \
    cc[N] = NTEST8(temp8);

static void op_INC_D() {
    temp16 = EA_DIRECT();
    BODY_INC(MemRead8(temp16));
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_INC_X() {
    temp16 = EA_INDEXED();
    BODY_INC(MemRead8(temp16));
    MemWrite8(temp8, temp16);
    CycleCounter += 6;
}
static void op_INC_E() {
    temp16 = EA_EXTENDED();
    BODY_INC(MemRead8(temp16));
    MemWrite8(temp8, temp16);
    CycleCounter += 7;
}
static void op_INCA() {
    BODY_INC(A_REG);
    A_REG = temp8;
    CycleCounter += 2;
}
static void op_INCB() {
    BODY_INC(B_REG);
    B_REG = temp8;
    CycleCounter += 2;
}

// --- TST: test byte (read-only, set flags) ---
#define BODY_TST(val) \
    cc[Z] = ZTEST(val); \
    cc[N] = NTEST8(val); \
    cc[V] = false;

static void op_TST_D() {
    temp8 = MemRead8(EA_DIRECT());
    BODY_TST(temp8);
    CycleCounter += 6;
}
static void op_TST_X() {
    temp16 = EA_INDEXED();
    temp8 = MemRead8(temp16);
    BODY_TST(temp8);
    CycleCounter += 6;
}
static void op_TST_E() {
    temp16 = EA_EXTENDED();
    temp8 = MemRead8(temp16);
    BODY_TST(temp8);
    CycleCounter += 7;
}
static void op_TSTA() {
    BODY_TST(A_REG);
    CycleCounter += 2;
}
static void op_TSTB() {
    BODY_TST(B_REG);
    CycleCounter += 2;
}

// --- CLR: clear byte ---
static void op_CLR_D() {
    MemWrite8(0, EA_DIRECT());
    cc[Z] = true; cc[N] = false; cc[V] = false; cc[C] = false;
    CycleCounter += 6;
}
static void op_CLR_X() {
    temp16 = EA_INDEXED();
    MemWrite8(0, temp16);
    cc[Z] = true; cc[N] = false; cc[V] = false; cc[C] = false;
    CycleCounter += 6;
}
static void op_CLR_E() {
    temp16 = EA_EXTENDED();
    MemWrite8(0, temp16);
    cc[Z] = true; cc[N] = false; cc[V] = false; cc[C] = false;
    CycleCounter += 7;
}
static void op_CLRA() {
    A_REG = 0;
    cc[Z] = true; cc[N] = false; cc[V] = false; cc[C] = false;
    CycleCounter += 2;
}
static void op_CLRB() {
    B_REG = 0;
    cc[Z] = true; cc[N] = false; cc[V] = false; cc[C] = false;
    CycleCounter += 2;
}

// --- JMP ---
static void op_JMP_D() {
    pc.Reg = dp.Reg | MemRead8(pc.Reg);
    CycleCounter += 3;
}
static void op_JMP_X() {
    pc.Reg = EA_INDEXED();
    CycleCounter += 3;
}
static void op_JMP_E() {
    pc.Reg = EA_EXTENDED();
    CycleCounter += 4;
}

// --- NOP ---
static void op_NOP() {
    CycleCounter += 2;
}

// --- Unimplemented/illegal opcode ---
static void op_ILLEGAL() {
    CycleCounter += 2;
}

// ============================================================
// Page 1 handler table (256 entries)
// Entries for Page2/Page3 prefixes and opcodes not yet extracted
// use a fallback that calls the original Do_Opcode switch logic.
// ============================================================

// Forward declare — these will be filled as handlers are added.
// For now, unimplemented opcodes point to op_ILLEGAL.
// The table is populated at the bottom of this file after all handlers
// are defined.
