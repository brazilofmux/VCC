/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).
Licensed under the GNU GPL v3 or later; see <http://www.gnu.org/licenses/>.
*/
#ifndef CHACHA_H
#define CHACHA_H

// ChaCha20 (RFC 8439), portable across CMOC (unsigned long is 32-bit)
// and host compilers (for the test-vector gate in tests/).

#if defined(_CMOC_VERSION_) || defined(__CMOC__)
typedef unsigned long  cc_u32;
#else
#include <stdint.h>
typedef uint32_t cc_u32;
#endif

// One 64-byte keystream block. key: 8 words, nonce: 3 words (RFC layout).
void chacha20_block(const cc_u32 key[8], cc_u32 counter,
                    const cc_u32 nonce[3], unsigned char out[64]);

// XOR a buffer with the keystream starting at block `counter`.
void chacha20_xor(const cc_u32 key[8], cc_u32 counter,
                  const cc_u32 nonce[3], unsigned char* data, unsigned len);

#endif
