/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).
Licensed under the GNU GPL v3 or later; see <http://www.gnu.org/licenses/>.
*/

// See os9sys.h. Each wrapper marshals through file-scope globals so
// the inline assembly can reference them by name, issues the SWI2
// syscall, and captures carry/B for the error path. CMOC's inline
// asm requires U and DP preserved; D, X, Y are ours to use.

#include "os9sys.h"

unsigned char os9_errno;

static const char*  gPath;
static unsigned char gByteA;
static unsigned char gByteB;
static void*        gBuf;
static unsigned     gCount;
static unsigned     gResult;
static unsigned char gErr;

int os9_create(const char* path, unsigned char attrs)
{
    gPath = path; gByteA = 2; gByteB = attrs; gErr = 0;
    asm
    {
        lda   :gByteA       // access mode: write
        ldb   :gByteB       // attributes
        ldx   :gPath
        swi2
        fcb   $83           // I$Create
        bcc   os9cr_ok
        stb   :gErr
        bra   os9cr_done
os9cr_ok:
        clrb
        stb   :gErr
        sta   :gByteA       // path number
os9cr_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return gByteA;
}

int os9_open(const char* path, unsigned char mode)
{
    gPath = path; gByteA = mode; gErr = 0;
    asm
    {
        lda   :gByteA
        ldx   :gPath
        swi2
        fcb   $84           // I$Open
        bcc   os9op_ok
        stb   :gErr
        bra   os9op_done
os9op_ok:
        clrb
        stb   :gErr
        sta   :gByteA
os9op_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return gByteA;
}

int os9_read(int pathnum, void* buf, unsigned count)
{
    gByteA = (unsigned char)pathnum; gBuf = buf; gCount = count; gErr = 0;
    // CMOC --os9 addresses globals through Y (the module data pointer),
    // and I$Read/I$Write pass and return the byte count IN Y - so the
    // count loads straight into Y via indexed addressing off the old Y
    // (never through D: LDD would clobber the path number in A), the
    // error test comes before TFR Y,D so B still holds the error code,
    // and Y is restored before any store on the error path.
    asm
    {
        lda   :gByteA
        ldx   :gBuf
        pshs  y
        ldy   :gCount
        swi2
        fcb   $89           // I$Read
        bcs   os9rd_err
        tfr   y,d
        puls  y
        std   :gResult
        clrb
        stb   :gErr
        bra   os9rd_done
os9rd_err:
        puls  y
        stb   :gErr
os9rd_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return (int)gResult;
}

int os9_readln(int pathnum, void* buf, unsigned count)
{
    gByteA = (unsigned char)pathnum; gBuf = buf; gCount = count; gErr = 0;
    asm
    {
        lda   :gByteA
        ldx   :gBuf
        pshs  y
        ldy   :gCount
        swi2
        fcb   $8B           // I$ReadLn
        bcs   os9rl_err
        tfr   y,d
        puls  y
        std   :gResult
        clrb
        stb   :gErr
        bra   os9rl_done
os9rl_err:
        puls  y
        stb   :gErr
os9rl_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return (int)gResult;
}

int os9_write(int pathnum, const void* buf, unsigned count)
{
    gByteA = (unsigned char)pathnum; gBuf = (void*)buf; gCount = count; gErr = 0;
    asm
    {
        lda   :gByteA
        ldx   :gBuf
        pshs  y
        ldy   :gCount
        swi2
        fcb   $8A           // I$Write
        bcs   os9wr_err
        tfr   y,d
        puls  y
        std   :gResult
        clrb
        stb   :gErr
        bra   os9wr_done
os9wr_err:
        puls  y
        stb   :gErr
os9wr_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return (int)gResult;
}

int os9_writln(int pathnum, const void* buf, unsigned count)
{
    gByteA = (unsigned char)pathnum; gBuf = (void*)buf; gCount = count; gErr = 0;
    asm
    {
        lda   :gByteA
        ldx   :gBuf
        pshs  y
        ldy   :gCount
        swi2
        fcb   $8C           // I$WritLn
        bcs   os9wl_err
        tfr   y,d
        puls  y
        std   :gResult
        clrb
        stb   :gErr
        bra   os9wl_done
os9wl_err:
        puls  y
        stb   :gErr
os9wl_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return (int)gResult;
}

int os9_close(int pathnum)
{
    gByteA = (unsigned char)pathnum; gErr = 0;
    asm
    {
        lda   :gByteA
        swi2
        fcb   $8F           // I$Close
        bcc   os9cl_ok
        stb   :gErr
        bra   os9cl_done
os9cl_ok:
        clrb
        stb   :gErr
os9cl_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return 0;
}

int os9_makdir(const char* path)
{
    gPath = path; gErr = 0;
    asm
    {
        lda   #$BF          // mode: dir + rwx
        ldb   #$1F          // attributes
        ldx   :gPath
        swi2
        fcb   $85           // I$MakDir
        bcc   os9md_ok
        stb   :gErr
        bra   os9md_done
os9md_ok:
        clrb
        stb   :gErr
os9md_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return 0;
}

int os9_time(unsigned char* buf)
{
    gBuf = buf; gErr = 0;
    asm
    {
        ldx   :gBuf
        swi2
        fcb   $15           // F$Time
        bcc   os9tm_ok
        stb   :gErr
        bra   os9tm_done
os9tm_ok:
        clrb
        stb   :gErr
os9tm_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return 0;
}

static unsigned gPosHi, gPosLo;

int os9_seek(int pathnum, unsigned long pos)
{
    gByteA = (unsigned char)pathnum;
    gPosHi = (unsigned)(pos >> 16);
    gPosLo = (unsigned)(pos & 0xFFFFu);
    gErr = 0;
    // I$Seek takes the 32-bit position in X:U. U is CMOC's frame
    // pointer, so it is saved around the call. The path number loads
    // into A last - LDD (for the position LSW) goes through A:B, so
    // loading A any earlier would leave the LSW high byte as the path.
    asm
    {
        ldx   :gPosHi
        ldd   :gPosLo
        pshs  u
        tfr   d,u
        lda   :gByteA
        swi2
        fcb   $88           // I$Seek
        puls  u
        bcc   os9sk_ok
        stb   :gErr
        bra   os9sk_done
os9sk_ok:
        clrb
        stb   :gErr
os9sk_done:
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return 0;
}

long os9_filesize(int pathnum)
{
    gByteA = (unsigned char)pathnum;
    gErr = 0;
    // I$GetStt SS.Size (code 2) returns the size in X:U. The error
    // test must come before tfr u,d or the transfer clobbers B (the
    // error code). Globals are Y-relative, so storing them while U is
    // still the syscall's LSW is fine; only CMOC's frame pointer needs
    // restoring before return.
    asm
    {
        lda   :gByteA
        ldb   #2            // SS.Size
        pshs  u
        swi2
        fcb   $8D           // I$GetStt
        bcs   os9fs_err
        tfr   u,d
        std   :gPosLo
        stx   :gPosHi
        clrb
        stb   :gErr
        bra   os9fs_out
os9fs_err:
        stb   :gErr
os9fs_out:
        puls  u
    }
    if (gErr) { os9_errno = gErr; return -1; }
    return ((long)gPosHi << 16) | (long)gPosLo;
}
