#pragma once
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

// host_services: the single seam between the portable core and the host OS.
//
// On Windows this is exactly <Windows.h> - zero behavior change for the
// MSBuild path. On other hosts it supplies the narrow slice of the Win32
// surface the portable core actually touches: opaque handle types,
// RECT/POINT, MessageBox routed to stderr, OutputDebugString, and inert
// clipboard stubs. The Windows-only shell (Vcc.cpp, config.cpp,
// DirectX/DirectSound, dialogs) is deliberately NOT covered here; that
// code is replaced wholesale by the SDL shell per docs/porting-macos.md.
//
// Core files should include this instead of <Windows.h>. Growing the
// non-Windows section is fine, but each addition should serve a file in
// the portable set - if shell code needs something, port the shell code
// instead.

#ifdef _WIN32

#include <Windows.h>

#else

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

// MSVC calling-convention keywords are meaningless off Windows.
#define __fastcall
#define __cdecl
#define CALLBACK

// MSVC-isms that appear in the portable core.
#define _inline inline
#define __int64 long long

typedef void*           HWND;
typedef void*           HINSTANCE;
typedef void*           HMODULE;
typedef void*           HANDLE;
typedef void*           HGLOBAL;
typedef void*           HDC;
typedef void*           HBITMAP;
typedef int32_t         LONG;
typedef uint32_t        DWORD;
typedef uint16_t        WORD;
typedef uint8_t         BYTE;
typedef int             BOOL;
typedef unsigned int    UINT;
typedef char*           LPSTR;
typedef const char*     LPCSTR;
typedef intptr_t        LPARAM;
typedef uintptr_t       WPARAM;
typedef intptr_t        LRESULT;

struct RECT  { LONG left; LONG top; LONG right; LONG bottom; };
struct POINT { LONG x; LONG y; };

// Same layout as the Win32 GUID (audio.h carries one in its device-
// selection interface; off Windows it is never dereferenced).
struct _GUID
{
    uint32_t Data1;
    uint16_t Data2;
    uint16_t Data3;
    uint8_t  Data4[8];
};
typedef _GUID GUID;

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#define CW_USEDEFAULT   ((int)0x80000000)
#define MAX_PATH        260

#define MB_OK               0x00000000u
#define MB_YESNO            0x00000004u
#define MB_ICONERROR        0x00000010u
#define MB_ICONWARNING      0x00000030u
#define MB_SETFOREGROUND    0x00010000u
#define MB_TOPMOST          0x00040000u
#define MB_TASKMODAL        0x00002000u
#define IDOK            1
#define IDYES           6
#define IDNO            7

inline void OutputDebugStringA(const char* s)
{
    std::fputs(s ? s : "", stderr);
}
#define OutputDebugString OutputDebugStringA

inline int MessageBoxA(HWND, const char* text, const char* caption, UINT)
{
    std::fprintf(stderr, "[%s] %s\n",
                 caption ? caption : "VCC", text ? text : "");
    return IDOK;
}
#define MessageBox MessageBoxA

// Clipboard: headless hosts have none. OpenClipboard reports failure so
// callers take their error paths; the allocation trio stays real so
// SetClipboard's unchecked memcpy(GlobalLock(hMem), ...) never faults.
#define CF_TEXT         1u
#define GMEM_MOVEABLE   2u

// File handles for the printer-capture path (mc6821.cpp): HANDLE is a
// FILE* in disguise, CreateFile maps the creation disposition onto an
// fopen mode, and the printer-monitor "console" is stdout. The capture
// feature works the same as on Windows.
#define INVALID_HANDLE_VALUE    ((HANDLE)(intptr_t)-1)
#define GENERIC_READ            0x80000000u
#define GENERIC_WRITE           0x40000000u
#define FILE_SHARE_READ         1u
#define CREATE_ALWAYS           2u
#define OPEN_EXISTING           3u
#define OPEN_ALWAYS             4u
#define FILE_ATTRIBUTE_NORMAL   0x80u
#define STD_OUTPUT_HANDLE       ((DWORD)-11)

inline HANDLE CreateFileA(const char* name, DWORD access, DWORD, void*,
                          DWORD disposition, DWORD, HANDLE)
{
    const char* mode;
    switch (disposition)
    {
    case CREATE_ALWAYS: mode = "w+b"; break;
    case OPEN_ALWAYS:   mode = (access & GENERIC_WRITE) ? "a+b" : "ab"; break;
    default:            mode = (access & GENERIC_WRITE) ? "r+b" : "rb"; break;
    }
    FILE* f = std::fopen(name, mode);
    return f ? (HANDLE)f : INVALID_HANDLE_VALUE;
}
#define CreateFile CreateFileA

inline BOOL WriteFile(HANDLE h, const void* data, DWORD len,
                      unsigned long* written, void*)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return FALSE;
    const size_t n = std::fwrite(data, 1, len, (FILE*)h);
    std::fflush((FILE*)h);
    if (written) *written = (unsigned long)n;
    return n == len;
}

inline BOOL CloseHandle(HANDLE h)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return FALSE;
    if ((FILE*)h == stdout)
        return TRUE;
    std::fclose((FILE*)h);
    return TRUE;
}

inline BOOL   AllocConsole()                { return TRUE; }
inline BOOL   FreeConsole()                 { return TRUE; }
inline HANDLE GetStdHandle(DWORD)           { return (HANDLE)stdout; }
inline BOOL   SetConsoleTitle(const char*)  { return TRUE; }

inline BOOL WriteConsoleA(HANDLE h, const void* data, DWORD len,
                          unsigned long* written, void*)
{
    FILE* f = (h == nullptr || h == INVALID_HANDLE_VALUE) ? stdout : (FILE*)h;
    const size_t n = std::fwrite(data, 1, len, f);
    std::fflush(f);
    if (written) *written = (unsigned long)n;
    return TRUE;
}
#define WriteConsole WriteConsoleA

// Wall-clock time for the Cloud9 RTC device: a real implementation, not
// a stub - the emulated clock should tell true time on every host.
struct SYSTEMTIME
{
    WORD wYear;
    WORD wMonth;
    WORD wDayOfWeek;
    WORD wDay;
    WORD wHour;
    WORD wMinute;
    WORD wSecond;
    WORD wMilliseconds;
};

// Setting the host clock from inside the emulator (Cloud9 RTC write-
// enable) is a Windows-only party trick; on POSIX it needs root and is
// the wrong thing to do anyway. Report failure, change nothing.
inline BOOL SetLocalTime(const SYSTEMTIME*) { return FALSE; }

inline void GetLocalTime(SYSTEMTIME* st)
{
    std::time_t t = std::time(nullptr);
    std::tm tm_buf {};
    localtime_r(&t, &tm_buf);
    st->wYear         = (WORD)(tm_buf.tm_year + 1900);
    st->wMonth        = (WORD)(tm_buf.tm_mon + 1);
    st->wDayOfWeek    = (WORD)tm_buf.tm_wday;
    st->wDay          = (WORD)tm_buf.tm_mday;
    st->wHour         = (WORD)tm_buf.tm_hour;
    st->wMinute       = (WORD)tm_buf.tm_min;
    st->wSecond       = (WORD)tm_buf.tm_sec;
    st->wMilliseconds = 0;
}

// DLL cartridge loading does not exist off Windows - carts are statically
// linked per docs/porting-macos.md. These keep the loader compiling; the
// DLL path reports failure and callers take their error paths.
inline HMODULE LoadLibraryA(const char*)           { return nullptr; }
#define LoadLibrary LoadLibraryA
inline HMODULE GetModuleHandleA(const char*)       { return nullptr; }
#define GetModuleHandle GetModuleHandleA
inline void*   GetProcAddress(HMODULE, const char*) { return nullptr; }
inline BOOL    FreeLibrary(HMODULE)                { return TRUE; }
inline DWORD   GetLastError()                      { return 0; }

// Path of the running executable (module handles other than null have no
// POSIX equivalent and fall back to the executable). Returns the length
// copied, or nSize when truncated - callers that grow-and-retry (see
// fileutil.cpp ModulePath) keep working.
inline DWORD GetModuleFileNameA(HMODULE, char* out, DWORD nSize)
{
    if (out == nullptr || nSize == 0)
        return 0;
    char buf[4096];
#ifdef __APPLE__
    uint32_t cap = sizeof(buf);
    if (_NSGetExecutablePath(buf, &cap) != 0)
        return 0;
#else
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0)
        return 0;
    buf[n] = '\0';
#endif
    size_t len = std::strlen(buf);
    if (len >= nSize)
    {
        std::memcpy(out, buf, nSize - 1);
        out[nSize - 1] = '\0';
        return nSize;
    }
    std::memcpy(out, buf, len + 1);
    return (DWORD)len;
}
#define GetModuleFileName GetModuleFileNameA

// INI-file config storage (the private-profile API). Real
// implementations in libcommon/src/util/host_profile.cpp - same file
// format and semantics as the Win32 API, so existing vcc.ini configs
// carry over between hosts.
DWORD GetPrivateProfileStringA(const char* app, const char* key,
                               const char* def, char* out, DWORD size,
                               const char* file);
UINT  GetPrivateProfileIntA(const char* app, const char* key, int def,
                            const char* file);
BOOL  WritePrivateProfileStringA(const char* app, const char* key,
                                 const char* value, const char* file);
#define GetPrivateProfileString   GetPrivateProfileStringA
#define GetPrivateProfileInt      GetPrivateProfileIntA
#define WritePrivateProfileString WritePrivateProfileStringA

inline BOOL    OpenClipboard(HWND)                 { return FALSE; }
inline BOOL    CloseClipboard()                    { return TRUE; }
inline BOOL    EmptyClipboard()                    { return TRUE; }
inline HANDLE  GetClipboardData(UINT)              { return nullptr; }
inline HGLOBAL GlobalAlloc(UINT, size_t n)         { return std::calloc(1, n ? n : 1); }
inline void*   GlobalLock(HGLOBAL h)               { return h; }
inline BOOL    GlobalUnlock(HGLOBAL)               { return TRUE; }
inline HANDLE  SetClipboardData(UINT, HGLOBAL h)   { std::free(h); return nullptr; }

#endif // _WIN32
