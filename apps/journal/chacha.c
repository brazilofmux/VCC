/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).
Licensed under the GNU GPL v3 or later; see <http://www.gnu.org/licenses/>.
*/

// RFC 8439 ChaCha20. Written for clarity and portability: the same
// file compiles under CMOC for the 6309 and under clang for the
// host-side test-vector gate (tests/chacha_host.c). Rotates are
// shift-pairs; CMOC's unsigned long is a true 32-bit type.

#include "chacha.h"

#define ROTL(v, n) (((v) << (n)) | ((v) >> (32 - (n))))

#define QR(a, b, c, d)            \
    a += b; d ^= a; d = ROTL(d, 16); \
    c += d; b ^= c; b = ROTL(b, 12); \
    a += b; d ^= a; d = ROTL(d, 8);  \
    c += d; b ^= c; b = ROTL(b, 7)

void chacha20_block(const cc_u32 key[8], cc_u32 counter,
                    const cc_u32 nonce[3], unsigned char out[64])
{
    cc_u32 s[16];
    cc_u32 x[16];
    unsigned char i;

    s[0] = 0x61707865UL; s[1] = 0x3320646eUL;
    s[2] = 0x79622d32UL; s[3] = 0x6b206574UL;
    for (i = 0; i < 8; ++i) s[4 + i] = key[i];
    s[12] = counter;
    s[13] = nonce[0]; s[14] = nonce[1]; s[15] = nonce[2];

    for (i = 0; i < 16; ++i) x[i] = s[i];
    for (i = 0; i < 10; ++i)
    {
        QR(x[0], x[4], x[8],  x[12]);
        QR(x[1], x[5], x[9],  x[13]);
        QR(x[2], x[6], x[10], x[14]);
        QR(x[3], x[7], x[11], x[15]);
        QR(x[0], x[5], x[10], x[15]);
        QR(x[1], x[6], x[11], x[12]);
        QR(x[2], x[7], x[8],  x[13]);
        QR(x[3], x[4], x[9],  x[14]);
    }
    for (i = 0; i < 16; ++i)
    {
        const cc_u32 v = x[i] + s[i];
        out[i * 4 + 0] = (unsigned char)(v & 0xFF);
        out[i * 4 + 1] = (unsigned char)((v >> 8) & 0xFF);
        out[i * 4 + 2] = (unsigned char)((v >> 16) & 0xFF);
        out[i * 4 + 3] = (unsigned char)((v >> 24) & 0xFF);
    }
}

void chacha20_xor(const cc_u32 key[8], cc_u32 counter,
                  const cc_u32 nonce[3], unsigned char* data, unsigned len)
{
    unsigned char block[64];
    unsigned off = 0;
    while (off < len)
    {
        unsigned n = len - off;
        unsigned char j;
        if (n > 64) n = 64;
        chacha20_block(key, counter, nonce, block);
        ++counter;
        for (j = 0; j < (unsigned char)n; ++j)
            data[off + j] ^= block[j];
        off += n;
    }
}
