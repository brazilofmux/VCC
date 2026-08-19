/*
Copyright 2026 by the VCC Project Contributors.
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

// CPU benchmark for the emulator core, compiled with CMOC for DECB.
// A sieve of Eratosthenes plus a byte-copy pass - stable compiled-C
// code shapes (counted loops, array walks) rather than hand-tuned ROM
// assembly, which is the workload the block JIT's trace machinery is
// aimed at. Prints its result so differential runs verify computation,
// and a completion marker the harness greps for.
//
// Build/run: tests/bench/run.sh (cmoc podman container + vcc-headless).

#include <cmoc.h>

#define SIEVE_SIZE 2048
#define PASSES     30

static unsigned char sieve[SIEVE_SIZE];
static unsigned char copybuf[SIEVE_SIZE];

int main()
{
    unsigned pass;
    unsigned i, j;
    unsigned count = 0;

    printf("BENCH START\n");

    for (pass = 0; pass < PASSES; ++pass)
    {
        memset(sieve, 1, SIEVE_SIZE);
        count = 0;
        for (i = 2; i < SIEVE_SIZE; ++i)
        {
            if (sieve[i])
            {
                ++count;
                for (j = i + i; j < SIEVE_SIZE; j += i)
                    sieve[j] = 0;
            }
        }
        memcpy(copybuf, sieve, SIEVE_SIZE);
    }

    printf("PRIMES %u PASSES %u\n", count, (unsigned)PASSES);
    printf("BENCH DONE\n");
    return 0;
}
