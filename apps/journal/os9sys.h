/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).
Licensed under the GNU GPL v3 or later; see <http://www.gnu.org/licenses/>.
*/
#ifndef OS9SYS_H
#define OS9SYS_H

// Thin OS-9 Level 2 system-call layer for CMOC (--os9 target).
// Returns: path number or byte count on success, -1 on error
// (os9_errno holds the OS-9 error code).

extern unsigned char os9_errno;

int  os9_create(const char* path, unsigned char attrs);   // write mode
int  os9_open(const char* path, unsigned char mode);      // 1=r 2=w 3=u
int  os9_read(int pathnum, void* buf, unsigned count);
int  os9_readln(int pathnum, void* buf, unsigned count);
int  os9_write(int pathnum, const void* buf, unsigned count);
int  os9_writln(int pathnum, const void* buf, unsigned count);
int  os9_close(int pathnum);
int  os9_makdir(const char* path);
// buf receives 6 bytes: year(-1900) month day hour minute second.
int  os9_time(unsigned char* buf);
// 32-bit seek and file-size (I$Seek / I$GetStt SS.Size).
int  os9_seek(int pathnum, unsigned long pos);
long os9_filesize(int pathnum);

#endif
