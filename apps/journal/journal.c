/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).
Licensed under the GNU GPL v3 or later; see <http://www.gnu.org/licenses/>.
*/

// journal - an encrypted daily journal for NitrOS-9, in the spirit of
// a certain teenage doctor's sign-off screen.
//
//   journal              write today's entry (end with "." on its own line)
//   journal read         read today's entries
//   journal read 20260819    read a specific day
//   journal -d /d1/J ...     keep the journal somewhere else
//                            (default /dd/JOURNAL)
//
// Entries are ChaCha20-encrypted (RFC 8439, verified against its test
// vector on the build host) with a key derived from your passphrase.
// The derivation is a cheap ChaCha self-feedback fold, not PBKDF2 -
// honest rating: keeps out siblings and casual snoops, not the NSA.
// Each entry record: "DHJ1" magic, 12-byte nonce (time + file offset),
// 2-byte length, ciphertext of "DOOGIE01" + your text. The inner magic
// authenticates the passphrase on read.
//
// Build and run: tools/coco-run --os9 --headless apps/journal/journal.c
// (coco-run picks up chacha.c/os9sys.c beside it automatically).

#include <cmoc.h>
#include "chacha.h"
#include "os9sys.h"

#define MAX_ENTRY 1536

static cc_u32 gKey[8];
static unsigned char gDir[64] = "/dd/JOURNAL";
static unsigned char gFile[96];
static unsigned char gPlain[MAX_ENTRY + 8];
static unsigned char gRecord[MAX_ENTRY + 32];

// Read one line from stdin via I$ReadLn: the SCF line editor handles
// backspace etc. and the count includes the terminating CR, which we
// strip. Returns the length, or -1 on EOF/error.
static int get_line(char* buf, unsigned max)
{
    int n = os9_readln(0, buf, max - 1);
    if (n <= 0)
        return -1;
    if (buf[n - 1] == '\r')
        --n;
    buf[n] = 0;
    return n;
}

static void derive_key(const char* pass)
{
    unsigned char kb[32];
    unsigned char i, r;
    cc_u32 nonce[3];
    unsigned len = strlen(pass);

    // Fold the passphrase into 32 bytes...
    for (i = 0; i < 32; ++i) kb[i] = (unsigned char)(i * 7 + 0x5A);
    for (i = 0; i < (unsigned char)len; ++i)
        kb[i & 31] ^= (unsigned char)(pass[i] + i);
    // ...then stir with ChaCha over itself a few rounds.
    nonce[0] = 0x4A4F5552UL; nonce[1] = 0x4E414C21UL; nonce[2] = (cc_u32)len;
    for (r = 0; r < 4; ++r)
    {
        for (i = 0; i < 8; ++i)
            gKey[i] = (cc_u32)kb[i*4] | ((cc_u32)kb[i*4+1] << 8) |
                      ((cc_u32)kb[i*4+2] << 16) | ((cc_u32)kb[i*4+3] << 24);
        chacha20_xor(gKey, r, nonce, kb, 32);
    }
    for (i = 0; i < 8; ++i)
        gKey[i] = (cc_u32)kb[i*4] | ((cc_u32)kb[i*4+1] << 8) |
                  ((cc_u32)kb[i*4+2] << 16) | ((cc_u32)kb[i*4+3] << 24);
}

static void date_stamp(unsigned char* t, char* out)   // YYYYMMDD
{
    sprintf(out, "%04u%02u%02u", 1900 + t[0], t[1], t[2]);
}

static void journal_path(const char* day)
{
    sprintf((char*)gFile, "%s/%s\r", (char*)gDir, day);
}

static int write_entry(void)
{
    unsigned char t[6];
    char day[12];
    char line[128];
    unsigned len = 8;
    unsigned i;
    int n, p;
    long size;
    cc_u32 nonce[3];

    if (os9_time(t) != 0) { printf("time? error %u\n", os9_errno); return 1; }
    date_stamp(t, day);

    printf("Journal for %04u/%02u/%02u. End with '.' alone.\n\n",
           1900 + t[0], t[1], t[2]);

    memcpy(gPlain, "DOOGIE01", 8);
    for (;;)
    {
        n = get_line(line, sizeof(line));
        if (n < 0)
            break;
        if (n == 1 && line[0] == '.')
            break;
        if (len + (unsigned)n + 1 > MAX_ENTRY)
        {
            printf("(entry full)\n");
            break;
        }
        memcpy(gPlain + len, line, (unsigned)n);
        len += (unsigned)n;
        gPlain[len++] = '\n';
    }
    if (len <= 8)
    {
        printf("Nothing to save.\n");
        return 0;
    }

    // Ensure the directory and open/create the day file for append.
    sprintf((char*)gFile, "%s\r", (char*)gDir);
    os9_makdir((char*)gFile);      // best effort; exists is fine
    journal_path(day);
    p = os9_open((char*)gFile, 3);
    if (p < 0)
        p = os9_create((char*)gFile, 0x03);
    if (p < 0) { printf("cannot open %s (error %u)\n", (char*)gFile, os9_errno); return 1; }
    size = os9_filesize(p);
    if (size < 0) size = 0;
    os9_seek(p, (unsigned long)size);

    // Nonce: wall time plus the append offset - unique per record.
    nonce[0] = ((cc_u32)t[0] << 16) | ((cc_u32)t[1] << 8) | t[2];
    nonce[1] = ((cc_u32)t[3] << 16) | ((cc_u32)t[4] << 8) | t[5];
    nonce[2] = (cc_u32)size;

    memcpy(gRecord, "DHJ1", 4);
    for (i = 0; i < 3; ++i)
    {
        gRecord[4 + i*4]     = (unsigned char)(nonce[i] & 0xFF);
        gRecord[5 + i*4]     = (unsigned char)((nonce[i] >> 8) & 0xFF);
        gRecord[6 + i*4]     = (unsigned char)((nonce[i] >> 16) & 0xFF);
        gRecord[7 + i*4]     = (unsigned char)((nonce[i] >> 24) & 0xFF);
    }
    gRecord[16] = (unsigned char)(len & 0xFF);
    gRecord[17] = (unsigned char)((len >> 8) & 0xFF);
    memcpy(gRecord + 18, gPlain, len);
    chacha20_xor(gKey, 1, nonce, gRecord + 18, len);

    if (os9_write(p, gRecord, 18 + len) != (int)(18 + len))
    {
        printf("write failed (error %u)\n", os9_errno);
        os9_close(p);
        return 1;
    }
    os9_close(p);
    printf("\nSaved (%u bytes, encrypted).\n", len - 8);
    return 0;
}

static int read_entries(const char* day)
{
    int p, got;
    unsigned len, i, count = 0;
    cc_u32 nonce[3];
    unsigned char hdr[18];

    journal_path(day);
    p = os9_open((char*)gFile, 1);
    if (p < 0)
    {
        printf("No journal for %s.\n", day);
        return 1;
    }
    for (;;)
    {
        got = os9_read(p, hdr, 18);
        if (got != 18)
            break;
        if (memcmp(hdr, "DHJ1", 4) != 0)
        {
            printf("(bad record - stopping)\n");
            break;
        }
        for (i = 0; i < 3; ++i)
            nonce[i] = (cc_u32)hdr[4+i*4] | ((cc_u32)hdr[5+i*4] << 8) |
                       ((cc_u32)hdr[6+i*4] << 16) | ((cc_u32)hdr[7+i*4] << 24);
        len = (unsigned)hdr[16] | ((unsigned)hdr[17] << 8);
        if (len > MAX_ENTRY)
        {
            printf("(bad length - stopping)\n");
            break;
        }
        if (os9_read(p, gPlain, len) != (int)len)
            break;
        chacha20_xor(gKey, 1, nonce, gPlain, len);
        if (memcmp(gPlain, "DOOGIE01", 8) != 0)
        {
            printf("(wrong passphrase)\n");
            os9_close(p);
            return 1;
        }
        ++count;
        printf("--- entry %u ---\n", count);
        gPlain[len] = 0;
        printf("%s", (char*)(gPlain + 8));
    }
    os9_close(p);
    if (count == 0)
        printf("(no entries)\n");
    return 0;
}

int main(int argc, char* argv[])
{
    char pass[64];
    const char* day = 0;
    unsigned char do_read = 0;
    int a = 1;
    unsigned char t[6];
    char today[12];

    while (a < argc)
    {
        if (strcmp(argv[a], "-d") == 0 && a + 1 < argc)
        {
            strcpy((char*)gDir, argv[a + 1]);
            a += 2;
        }
        else if (strcmp(argv[a], "read") == 0)
        {
            do_read = 1;
            ++a;
            if (a < argc) { day = argv[a]; ++a; }
        }
        else
            ++a;
    }

    // Period-appropriate presentation: white on blue. Sent with
    // I$Write, not printf - the fg-white parameter is a NUL byte,
    // which would terminate a printf format string early and leave
    // the escape parser eating the next printable character.
    {
        static const unsigned char dress[7] =
            { 0x1B, 0x32, 0x00,     // foreground: white
              0x1B, 0x33, 0x01,     // background: blue
              0x0C };               // clear screen
        os9_write(1, dress, 7);
    }
    printf("PERSONAL JOURNAL\n================\n");

    printf("Passphrase: ");
    if (get_line(pass, sizeof(pass)) <= 0)
    {
        printf("No passphrase, no journal.\n");
        return 1;
    }
    derive_key(pass);

    if (do_read)
    {
        if (day == 0)
        {
            if (os9_time(t) != 0) { printf("time? error %u\n", os9_errno); return 1; }
            date_stamp(t, today);
            day = today;
        }
        return read_entries(day);
    }
    return write_entry();
}
