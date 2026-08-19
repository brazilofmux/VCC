// Syscall-layer probe: time, mkdir, create/write, open/read roundtrip.
#include <cmoc.h>
#include "os9sys.h"

int main()
{
    unsigned char t[6];
    char buf[64];
    int p, n;

    if (os9_time(t) == 0)
        printf("TIME %u/%u/%u %u:%u:%u\n",
               1900 + t[0], t[1], t[2], t[3], t[4], t[5]);
    else
        printf("TIME ERR %u\n", os9_errno);

    os9_makdir("/d1/PROBE\r");   // may already exist; ignore error

    p = os9_create("/d1/PROBE/test\r", 0x03);
    if (p < 0) { printf("CREATE ERR %u\n", os9_errno); return 1; }
    os9_write(p, "SECRET-ROUNDTRIP", 16);
    os9_close(p);

    p = os9_open("/d1/PROBE/test\r", 1);
    if (p < 0) { printf("OPEN ERR %u\n", os9_errno); return 1; }
    n = os9_read(p, buf, 16);
    os9_close(p);
    buf[n > 0 ? n : 0] = 0;
    printf("READBACK [%s] (%d)\n", buf, n);
    return 0;
}
