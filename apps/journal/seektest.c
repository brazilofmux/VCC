/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).
Licensed under the GNU GPL v3 or later; see <http://www.gnu.org/licenses/>.
*/

// Isolation probe for the create -> filesize -> seek -> write sequence
// that journal.c uses for appends. Prints every intermediate value.

#include <cmoc.h>
#include "os9sys.h"

int main(void)
{
    int p, r;
    long sz;

    p = os9_create("/d1/SEEKTEST\r", 0x03);
    printf("CREATE P=%d E=%u\n", p, os9_errno);
    if (p < 0) return 1;

    sz = os9_filesize(p);
    printf("SIZE %ld E=%u\n", sz, os9_errno);

    r = os9_seek(p, 0UL);
    printf("SEEK R=%d E=%u\n", r, os9_errno);

    r = os9_write(p, "HELLO-SEEK-WORLD", 16);
    printf("WRITE R=%d E=%u\n", r, os9_errno);

    sz = os9_filesize(p);
    printf("SIZE2 %ld E=%u\n", sz, os9_errno);
    os9_close(p);
    return 0;
}
